#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0
#
# Post-processing for the FANUC motion-planner benchmark. This script touches no
# MoveIt API: it only reads the rosbag recorded by benchmark.launch.xml and
#   1. reports the per-MOTR begin->goal elapsed times and aggregate statistics
#      (count / mean / stdev / longest / shortest) and writes them to a CSV, and
#   2. plots the time-series of planned vs. actual joint positions and the
#      collaborative speed scaling, saving the figures as PNGs.
#
# Usage:
#   ros2 run fanuc_benchmark analyze_benchmark.py <bag_path> [--out-dir DIR]
#                                                 [--no-show] [--log-level LEVEL]
#   # or, if not installed:
#   python3 analyze_benchmark.py <bag_path>
#
# <bag_path> is the directory created by `ros2 bag record -o ...`
# (e.g. benchmark_20260619_101530). If omitted, the newest benchmark_* bag in
# the current directory is used.

import argparse
import glob
import json
import logging
import math
import os
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py

logger = logging.getLogger("analyze_benchmark")

# Topics recorded by benchmark.launch.xml.
TOPIC_MOTR = "/motion_task_test/motr_result"
TOPIC_CONTROLLER_STATE = "/joint_trajectory_controller/controller_state"
TOPIC_JOINT_STATES = "/joint_states"
TOPIC_SPEED_SCALING = "/fanuc_gpio_controller/collaborative_speed_scaling"


def find_latest_bag():
    candidates = sorted(c for c in glob.glob("benchmark_*") if os.path.isdir(c))
    if not candidates:
        sys.exit("No benchmark_* bag found in the current directory; pass a bag path explicitly.")
    return candidates[-1]


def open_reader(bag_path):
    """Open a rosbag2 directory, auto-detecting the storage id (sqlite3/mcap)."""
    metadata = os.path.join(bag_path, "metadata.yaml")
    storage_id = "sqlite3"
    if os.path.exists(metadata):
        with open(metadata, "r") as f:
            if "mcap" in f.read():
                storage_id = "mcap"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, type_map


def read_messages(bag_path, wanted_topics):
    """Yield (topic, deserialized_msg, t_nanoseconds) for the wanted topics."""
    reader, type_map = open_reader(bag_path)
    msg_classes = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in wanted_topics or topic not in type_map:
            continue
        if topic not in msg_classes:
            msg_classes[topic] = get_message(type_map[topic])
        yield topic, deserialize_message(data, msg_classes[topic]), t


# --------------------------------------------------------------------------
# Timing statistics
# --------------------------------------------------------------------------
def compute_stats(values):
    if not values:
        return None
    n = len(values)
    mean = sum(values) / n
    stdev = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    return dict(count=n, mean=mean, stdev=stdev, longest=max(values), shortest=min(values))


def log_stats(title, stats):
    if stats is None:
        logger.info("  %-8s: no successful MOTRs", title)
        return
    logger.info(
        "  %-8s: n=%d  mean=%.3fs  stdev=%.3fs  longest=%.3fs  shortest=%.3fs",
        title,
        stats["count"],
        stats["mean"],
        stats["stdev"],
        stats["longest"],
        stats["shortest"],
    )


def process_timings(bag_path, out_dir):
    motrs = []
    for _topic, msg, _t in read_messages(bag_path, {TOPIC_MOTR}):
        try:
            motrs.append(json.loads(msg.data))
        except (json.JSONDecodeError, AttributeError):
            logger.warning("Skipping unparseable %s message", TOPIC_MOTR)
    motrs.sort(key=lambda m: m.get("index", 0))

    if not motrs:
        logger.warning("No '%s' messages in the bag; skipping timing report.", TOPIC_MOTR)
        return

    planner = motrs[0].get("planner", "unknown")
    logger.info("================ MOTION-TASK TIMING (planner=%s) ================", planner)
    logger.info("Per-MOTR begin->goal elapsed time:")
    for m in motrs:
        logger.info(
            "  MOTR %2d [%s] %-4s total=%.3fs (plan=%.3f exec=%.3f)",
            m.get("index"),
            m.get("direction"),
            "OK" if m.get("success") else "FAIL",
            m.get("total_s", 0.0),
            m.get("plan_s", 0.0),
            m.get("exec_s", 0.0),
        )

    def totals(direction):
        return [
            m["total_s"]
            for m in motrs
            if m.get("success") and (direction is None or m.get("direction") == direction)
        ]

    logger.info("Aggregate over successful MOTRs:")
    log_stats("Forward", compute_stats(totals("FWD")))
    log_stats("Reverse", compute_stats(totals("REV")))
    log_stats("All", compute_stats(totals(None)))
    logger.info("=======================================================================")

    csv_path = os.path.join(out_dir, "benchmark_timings.csv")
    with open(csv_path, "w") as f:
        f.write("motr_index,direction,planner,success,plan_s,exec_s,total_s,segments\n")
        for m in motrs:
            f.write(
                f"{m.get('index')},{m.get('direction')},{m.get('planner')},"
                f"{1 if m.get('success') else 0},{m.get('plan_s')},{m.get('exec_s')},"
                f"{m.get('total_s')},{m.get('segments')}\n"
            )
    logger.info("Wrote per-MOTR timings to %s", csv_path)


# --------------------------------------------------------------------------
# Time-series plots
# --------------------------------------------------------------------------
def process_plots(bag_path, out_dir, show):
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    joint_names = None
    t0 = None

    # Planned == controller_state.reference, actual == controller_state.feedback.
    planned = {"t": [], "pos": []}
    actual = {"t": [], "pos": []}
    scaling = {"t": [], "val": []}

    wanted = {TOPIC_CONTROLLER_STATE, TOPIC_SPEED_SCALING}
    for topic, msg, t in read_messages(bag_path, wanted):
        ts = t * 1e-9
        if t0 is None:
            t0 = ts
        rel = ts - t0
        if topic == TOPIC_CONTROLLER_STATE:
            if joint_names is None and msg.joint_names:
                joint_names = list(msg.joint_names)
            if msg.reference.positions:
                planned["t"].append(rel)
                planned["pos"].append(list(msg.reference.positions))
            if msg.feedback.positions:
                actual["t"].append(rel)
                actual["pos"].append(list(msg.feedback.positions))
        elif topic == TOPIC_SPEED_SCALING:
            scaling["t"].append(rel)
            scaling["val"].append(float(msg.collaborative_speed_scaling))

    saved = []

    # Joints: planned vs actual, one subplot per joint.
    if joint_names and (planned["pos"] or actual["pos"]):
        n = len(joint_names)
        fig, axes = plt.subplots(n, 1, figsize=(11, 2.2 * n), sharex=True)
        if n == 1:
            axes = [axes]
        for j, name in enumerate(joint_names):
            ax = axes[j]
            if planned["pos"]:
                ax.plot(planned["t"], [p[j] for p in planned["pos"]], label="planned (reference)", lw=1.3)
            if actual["pos"]:
                ax.plot(actual["t"], [p[j] for p in actual["pos"]], label="actual (feedback)", lw=1.0, alpha=0.85)
            ax.set_ylabel(f"{name}\n[rad]")
            ax.grid(True, alpha=0.3)
            if j == 0:
                ax.legend(loc="upper right", fontsize=8)
        axes[-1].set_xlabel("time [s]")
        fig.suptitle("Joint positions: planned vs. actual")
        fig.tight_layout()
        path = os.path.join(out_dir, "benchmark_joints.png")
        fig.savefig(path, dpi=120)
        saved.append(path)
    else:
        logger.warning(
            "No data on %s; cannot plot planned-vs-actual joints. "
            "Is the joint_trajectory_controller publishing controller_state?",
            TOPIC_CONTROLLER_STATE,
        )

    # Collaborative speed scaling.
    if scaling["val"]:
        fig, ax = plt.subplots(figsize=(11, 3))
        ax.plot(scaling["t"], scaling["val"], lw=1.3, color="tab:red")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("speed scaling")
        ax.set_title("fanuc_gpio_controller/collaborative_speed_scaling")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        path = os.path.join(out_dir, "benchmark_speed_scaling.png")
        fig.savefig(path, dpi=120)
        saved.append(path)
    else:
        logger.warning("No data on %s; cannot plot speed scaling.", TOPIC_SPEED_SCALING)

    for p in saved:
        logger.info("Saved plot %s", p)
    if show and saved:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analyze a FANUC motion-planner benchmark rosbag.")
    parser.add_argument("bag", nargs="?", help="Path to the rosbag directory (default: newest benchmark_* in CWD).")
    parser.add_argument("--out-dir", help="Where to write the CSV and PNGs (default: alongside the bag).")
    parser.add_argument("--no-show", action="store_true", help="Do not open plot windows; only save PNGs.")
    parser.add_argument(
        "--log-level",
        default=os.environ.get("ANALYZE_BENCHMARK_LOG_LEVEL", "INFO"),
        help="Logging level: DEBUG, INFO, WARNING, ERROR (default: INFO).",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(levelname)s: %(message)s",
    )

    bag_path = args.bag or find_latest_bag()
    if not os.path.isdir(bag_path):
        sys.exit(f"Bag path '{bag_path}' is not a directory.")
    out_dir = args.out_dir or bag_path
    os.makedirs(out_dir, exist_ok=True)

    logger.info("Reading bag: %s", bag_path)
    process_timings(bag_path, out_dir)
    process_plots(bag_path, out_dir, show=not args.no_show)


if __name__ == "__main__":
    main()
