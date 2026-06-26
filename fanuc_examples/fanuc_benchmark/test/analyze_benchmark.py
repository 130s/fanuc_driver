#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0
#
# Post-processing for the FANUC motion-planner benchmark. This script touches no
# MoveIt API: it only reads the rosbag recorded by benchmark.launch.xml and
#   1. reports the per-MOTR begin->goal elapsed times and aggregate statistics
#      (count / mean / stdev / longest / shortest) and writes them to a CSV,
#   2. reports the collaborative speed-scaling average / max / min / stdev and
#      the STMO SwitchControlState service-call count in the summary table and
#      writes them to CSVs,
#   3. writes the time-lapsed J1..J6 joint positions from /joint_states (actual)
#      and /display_planned_path (the motion planner's plan) to CSVs so they can
#      be plotted over time, and
#   4. plots the time-series of planned vs. actual joint positions and the
#      collaborative speed scaling, saving the figures as PNGs.
#
# All artifacts share the benchmark_fanuc-driver* base name (OUTPUT_PREFIX).
#
# Usage:
#   ros2 run fanuc_benchmark analyze_benchmark.py <bag_path> [--out-dir DIR]
#                                                 [--no-show] [--log-level LEVEL]
#   # or, if not installed:
#   python3 analyze_benchmark.py <bag_path>
#
# <bag_path> is the directory created by `ros2 bag record -o ...`
# (e.g. benchmark_fanuc-driver_20260619_101530). If omitted, the newest
# benchmark_* bag in the current directory is used.

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
TOPIC_DISPLAY_PATH = "/display_planned_path"
TOPIC_STMO_CALLS = "/fanuc_benchmark/stmo_service_call_count"

# Base name shared by every artifact this script writes (CSVs + PNGs), so the
# whole benchmark output is identifiable at a glance: benchmark_fanuc-driver.csv,
# benchmark_fanuc-driver_joint_states.csv, ...
OUTPUT_PREFIX = "benchmark_fanuc-driver"


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


def bag_t0(bag_path):
    """Earliest message timestamp in the bag, in seconds.

    Used as the common time origin so every time-series CSV/PNG shares one t=0
    and the joint_states trace, the planner's plan and the speed scaling can be
    overlaid on the same time axis. rosbag2 returns messages in time order, so
    the first message read is the earliest."""
    reader, _ = open_reader(bag_path)
    if reader.has_next():
        _topic, _data, t = reader.read_next()
        return t * 1e-9
    return 0.0


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

    # Each MOTR is recorded only after it completes (success or fail), so the bag
    # tells us what ran and finished. A MOTR that hung mid-execution never gets
    # published, leaving a gap in the index sequence -- detect that so "what did
    # NOT finish" is answerable straight from this report.
    recorded = len(motrs)
    succeeded = sum(1 for m in motrs if m.get("success"))
    failed = recorded - succeeded
    indices = [m.get("index", 0) for m in motrs]
    max_index = max(indices) if indices else 0
    missing = sorted(set(range(1, max_index + 1)) - set(indices))

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

    # Headline so it is easy to see at a glance what ran, finished, and what did not.
    logger.info("---------------------------------------------------------------")
    logger.info("Finished MOTRs: %d recorded  (%d OK, %d FAIL)", recorded, succeeded, failed)
    if missing:
        logger.warning(
            "Did NOT finish: MOTR(s) %s never completed (no result recorded) -- "
            "the runner likely hung or was interrupted mid-execution.",
            ", ".join(str(i) for i in missing),
        )
    if failed == 0 and not missing:
        logger.info("RESULT: all %d recorded MOTR(s) finished successfully.", recorded)
    else:
        logger.warning("RESULT: INCOMPLETE -- %d failed, %d never finished.", failed, len(missing))

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

    csv_path = os.path.join(out_dir, f"{OUTPUT_PREFIX}.csv")
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
# Collaborative speed-scaling statistics
# --------------------------------------------------------------------------
def process_speed_scaling_stats(bag_path, out_dir):
    """Aggregate /fanuc_gpio_controller/collaborative_speed_scaling over the run
    and report average / max / min / stdev in the summary table + a CSV."""
    values = [
        float(msg.collaborative_speed_scaling)
        for _topic, msg, _t in read_messages(bag_path, {TOPIC_SPEED_SCALING})
    ]

    logger.info("================ COLLABORATIVE SPEED SCALING ==================")
    logger.info("Topic: %s", TOPIC_SPEED_SCALING)
    if not values:
        logger.warning("No '%s' messages in the bag; skipping speed-scaling stats.", TOPIC_SPEED_SCALING)
        logger.info("=======================================================================")
        return

    n = len(values)
    avg = sum(values) / n
    stdev = math.sqrt(sum((v - avg) ** 2 for v in values) / (n - 1)) if n > 1 else 0.0
    vmax = max(values)
    vmin = min(values)
    logger.info(
        "  n=%d  avg=%.4f  max=%.4f  min=%.4f  stdev=%.4f",
        n, avg, vmax, vmin, stdev,
    )
    logger.info("=======================================================================")

    csv_path = os.path.join(out_dir, f"{OUTPUT_PREFIX}_speed_scaling_stats.csv")
    with open(csv_path, "w") as f:
        f.write("topic,count,average,max,min,stdev\n")
        f.write(f"{TOPIC_SPEED_SCALING},{n},{avg},{vmax},{vmin},{stdev}\n")
    logger.info("Wrote speed-scaling stats to %s", csv_path)


# --------------------------------------------------------------------------
# STMO SwitchControlState service-call count
# --------------------------------------------------------------------------
def process_stmo_service_calls(bag_path, out_dir):
    """Report how many times stmo_recovery.py had to call the
    fanuc_msgs/srv/SwitchControlState service. stmo_recovery.py publishes the
    running total on TOPIC_STMO_CALLS, so the last value recorded is the final
    count. No messages => the workaround was never needed (0 calls)."""
    counts = [int(msg.data) for _topic, msg, _t in read_messages(bag_path, {TOPIC_STMO_CALLS})]
    # The publisher only ever increments, so the maximum is the final total even
    # if messages arrive out of order in the bag.
    total = max(counts) if counts else 0

    logger.info("================ STMO RECOVERY SERVICE CALLS ==================")
    logger.info("Service: fanuc_msgs/srv/SwitchControlState (%s)", TOPIC_STMO_CALLS)
    logger.info("  SwitchControlState service calls: %d", total)
    logger.info("=======================================================================")

    csv_path = os.path.join(out_dir, f"{OUTPUT_PREFIX}_stmo_service_calls.csv")
    with open(csv_path, "w") as f:
        f.write("service,switch_control_state_calls\n")
        f.write(f"fanuc_msgs/srv/SwitchControlState,{total}\n")
    logger.info("Wrote STMO service-call count to %s", csv_path)


# --------------------------------------------------------------------------
# Joint position time-series (for later plotting)
# --------------------------------------------------------------------------
def _write_joint_timeseries_csv(csv_path, leading_cols, joint_names, rows):
    """rows: list of (leading_values_tuple, {joint_name: position})."""
    with open(csv_path, "w") as f:
        f.write(",".join(leading_cols + joint_names) + "\n")
        for leading, pos_by_name in rows:
            cells = [f"{v}" for v in leading]
            cells += [("" if joint not in pos_by_name else f"{pos_by_name[joint]}") for joint in joint_names]
            f.write(",".join(cells) + "\n")


def process_joint_timeseries(bag_path, out_dir, t0):
    """Write the time-lapsed J1..J6 positions from two sources, so they can be
    plotted over time later:
      * /joint_states                 -> actual joint positions as they evolve,
      * /display_planned_path          -> the motion planner's plan (each planned
                                          trajectory's waypoints, timestamped at
                                          plan-publish time + point time_from_start).
    Both share the bag's t0 so the actual trace and the plan line up in time."""
    js_names = None
    js_rows = []  # (time_s,), {name: pos}

    plan_names = None
    plan_rows = []  # (time_s, plan_index, point_index), {name: pos}
    plan_index = 0

    for topic, msg, t in read_messages(bag_path, {TOPIC_JOINT_STATES, TOPIC_DISPLAY_PATH}):
        rel = t * 1e-9 - t0
        if topic == TOPIC_JOINT_STATES:
            if js_names is None and msg.name:
                js_names = list(msg.name)
            js_rows.append(((f"{rel:.6f}",), dict(zip(msg.name, msg.position))))
        elif topic == TOPIC_DISPLAY_PATH:
            plan_index += 1
            for robot_traj in msg.trajectory:
                jt = robot_traj.joint_trajectory
                if not jt.joint_names:
                    continue
                if plan_names is None:
                    plan_names = list(jt.joint_names)
                for pidx, point in enumerate(jt.points):
                    tfs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                    plan_rows.append(
                        ((f"{rel + tfs:.6f}", plan_index, pidx), dict(zip(jt.joint_names, point.positions)))
                    )

    if js_names and js_rows:
        path = os.path.join(out_dir, f"{OUTPUT_PREFIX}_joint_states.csv")
        _write_joint_timeseries_csv(path, ["time_s"], js_names, js_rows)
        logger.info("Wrote joint_states time-series (%d samples) to %s", len(js_rows), path)
    else:
        logger.warning("No data on %s; cannot write joint_states time-series.", TOPIC_JOINT_STATES)

    if plan_names and plan_rows:
        path = os.path.join(out_dir, f"{OUTPUT_PREFIX}_planned_path.csv")
        _write_joint_timeseries_csv(path, ["time_s", "plan_index", "point_index"], plan_names, plan_rows)
        logger.info(
            "Wrote motion planner plan time-series (%d points across %d plans) to %s",
            len(plan_rows), plan_index, path,
        )
    else:
        logger.warning("No data on %s; cannot write planner plan time-series.", TOPIC_DISPLAY_PATH)


# --------------------------------------------------------------------------
# Combined overview plot (one image)
# --------------------------------------------------------------------------
def process_combined_plot(bag_path, out_dir, t0, show):
    """One image, shared time axis from launch start to end, stacking:
      * collaborative_speed_scaling over time, with avg / max / min reference
        lines and a +/-stdev band, and
      * J1..J6 positions from /joint_states (actual, line) overlaid with the
        motion planner's plan from /display_planned_path (planned, points)."""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scaling = {"t": [], "val": []}
    js_names = None
    js = {"t": [], "pos": []}        # actual joint positions (dicts name->pos)
    plan_names = None
    plan = {"t": [], "pos": []}      # planner plan positions (dicts name->pos)

    wanted = {TOPIC_SPEED_SCALING, TOPIC_JOINT_STATES, TOPIC_DISPLAY_PATH}
    for topic, msg, t in read_messages(bag_path, wanted):
        rel = t * 1e-9 - t0
        if topic == TOPIC_SPEED_SCALING:
            scaling["t"].append(rel)
            scaling["val"].append(float(msg.collaborative_speed_scaling))
        elif topic == TOPIC_JOINT_STATES:
            if js_names is None and msg.name:
                js_names = list(msg.name)
            js["t"].append(rel)
            js["pos"].append(dict(zip(msg.name, msg.position)))
        elif topic == TOPIC_DISPLAY_PATH:
            for robot_traj in msg.trajectory:
                jt = robot_traj.joint_trajectory
                if not jt.joint_names:
                    continue
                if plan_names is None:
                    plan_names = list(jt.joint_names)
                for point in jt.points:
                    tfs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                    plan["t"].append(rel + tfs)
                    plan["pos"].append(dict(zip(jt.joint_names, point.positions)))

    joints = js_names or plan_names
    if not joints and not scaling["val"]:
        logger.warning("Nothing to plot for the combined overview (no joint or speed-scaling data).")
        return

    joints = joints or []
    n_panels = len(joints) + 1  # speed scaling on top, then one row per joint
    fig, axes = plt.subplots(n_panels, 1, figsize=(12, 2.0 * n_panels), sharex=True)
    if n_panels == 1:
        axes = [axes]

    # --- speed scaling with stat overlays ---
    ax = axes[0]
    if scaling["val"]:
        vals = scaling["val"]
        nval = len(vals)
        avg = sum(vals) / nval
        stdev = math.sqrt(sum((v - avg) ** 2 for v in vals) / (nval - 1)) if nval > 1 else 0.0
        vmax, vmin = max(vals), min(vals)
        ax.plot(scaling["t"], vals, lw=1.2, color="tab:red", label="speed scaling")
        span = [scaling["t"][0], scaling["t"][-1]]
        ax.fill_between(span, avg - stdev, avg + stdev, color="tab:gray", alpha=0.15,
                        label=f"±stdev ({stdev:.3f})")
        ax.axhline(avg, color="tab:blue", lw=1.2, label=f"avg ({avg:.3f})")
        ax.axhline(vmax, color="tab:green", ls="--", lw=1.0, label=f"max ({vmax:.3f})")
        ax.axhline(vmin, color="tab:orange", ls="--", lw=1.0, label=f"min ({vmin:.3f})")
        ax.legend(loc="upper right", fontsize=7, ncol=2)
    else:
        ax.text(0.5, 0.5, f"no data on {TOPIC_SPEED_SCALING}", ha="center", va="center", transform=ax.transAxes)
    ax.set_ylabel("speed\nscaling")
    ax.set_title("collaborative_speed_scaling (avg / max / min / ±stdev) + joint positions over time")
    ax.grid(True, alpha=0.3)

    # --- one row per joint: actual (joint_states) vs planner plan ---
    for j, name in enumerate(joints):
        ax = axes[j + 1]
        if js["pos"]:
            jt = [(tt, p[name]) for tt, p in zip(js["t"], js["pos"]) if name in p]
            if jt:
                ax.plot([x[0] for x in jt], [x[1] for x in jt], lw=1.0, color="tab:blue",
                        label="actual (joint_states)")
        if plan["pos"]:
            pt = [(tt, p[name]) for tt, p in zip(plan["t"], plan["pos"]) if name in p]
            if pt:
                ax.plot([x[0] for x in pt], [x[1] for x in pt], ".", ms=3, color="tab:red",
                        label="planned (planner plan)")
        ax.set_ylabel(f"{name}\n[rad]")
        ax.grid(True, alpha=0.3)
        if j == 0:
            ax.legend(loc="upper right", fontsize=7)
    axes[-1].set_xlabel("time since launch start [s]")

    fig.tight_layout()
    path = os.path.join(out_dir, f"{OUTPUT_PREFIX}_overview.png")
    fig.savefig(path, dpi=120)
    logger.info("Saved combined overview plot %s", path)
    if show:
        plt.show()


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
        path = os.path.join(out_dir, f"{OUTPUT_PREFIX}_joints.png")
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
        path = os.path.join(out_dir, f"{OUTPUT_PREFIX}_speed_scaling.png")
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
    t0 = bag_t0(bag_path)
    process_timings(bag_path, out_dir)
    process_speed_scaling_stats(bag_path, out_dir)
    process_stmo_service_calls(bag_path, out_dir)
    process_joint_timeseries(bag_path, out_dir, t0)
    process_combined_plot(bag_path, out_dir, t0, show=not args.no_show)
    process_plots(bag_path, out_dir, show=not args.no_show)


if __name__ == "__main__":
    main()
