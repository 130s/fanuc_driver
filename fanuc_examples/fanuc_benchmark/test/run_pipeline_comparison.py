#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0
#
# Sequential multi-pipeline benchmark orchestrator.
#
# Driven by benchmark_setup.launch.xml. For each requested planning pipeline it
# runs the existing single-run benchmark.launch.xml as a SEPARATE, clean
# relaunch -- so every pipeline gets its own correct move_group config (Pilz, in
# particular, gets its own joint-limits file) and the FANUC driver is brought up
# fresh each time. Each run moves the arm to the start pose before timing (the
# motion_task_test node does this), which is also the "return to start between
# pipelines" step.
#
# All runs share ONE parent run folder; each pipeline's bag + per-run analysis
# lands in a per-pipeline subfolder:
#
#   <output_dir>/benchmark_fanuc-driver_<timestamp>/
#       benchmark_fanuc-driver_mtc/      benchmark_fanuc-driver.csv, *.png, <bag>
#       benchmark_fanuc-driver_pilz_lin/ ...
#       benchmark_fanuc-driver_ompl/     ...
#
# When all pipelines have run, compare_benchmarks.py aggregates them into the
# headline, pipeline-labeled deliverables at the parent level
# (benchmark_fanuc-driver_comparison.csv / _comparison_stats.csv / _comparison.png).

import argparse
import logging
import os
import subprocess
import sys
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare_benchmarks  # noqa: E402  (sibling helper installed alongside this file)

logger = logging.getLogger("run_pipeline_comparison")

PREFIX = "benchmark_fanuc-driver"

# pipeline label -> (planner backend, move_group planning_pipeline, joint-limits file).
# mtc and ompl both use the ompl move_group pipeline; pilz_lin needs its own
# pipeline AND its own joint-limits file.
PIPELINE_CONFIG = {
    "mtc": ("mtc", "ompl", "joint_limits.yaml"),
    "ompl": ("ompl", "ompl", "joint_limits.yaml"),
    "pilz_lin": ("pilz_lin", "pilz_industrial_motion_planner", "pilz_joint_limits.yaml"),
}


def joint_limits_path(filename):
    return os.path.join(get_package_share_directory("fanuc_moveit_config"), "config", filename)


def run_one_pipeline(label, args, parent_dir):
    """Launch benchmark.launch.xml for a single pipeline; block until it finishes."""
    if label not in PIPELINE_CONFIG:
        logger.error("Unknown pipeline '%s' (expected one of: %s); skipping.", label, ", ".join(PIPELINE_CONFIG))
        return False
    planner, planning_pipeline, jl_file = PIPELINE_CONFIG[label]

    cmd = [
        "ros2", "launch", "fanuc_benchmark", "benchmark.launch.xml",
        f"planner:={planner}",
        f"planning_pipeline:={planning_pipeline}",
        f"joint_limits_file:={joint_limits_path(jl_file)}",
        f"output_dir:={parent_dir}",
        # bag becomes <parent_dir>/benchmark_fanuc-driver_<label>
        f"timestamp:={label}",
        f"robot_model:={args.robot_model}",
        f"robot_ip:={args.robot_ip}",
        f"iterations:={args.iterations}",
        f"velocity_scaling:={args.velocity_scaling}",
        f"acceleration_scaling:={args.acceleration_scaling}",
        f"planning_time:={args.planning_time}",
        f"launch_rviz:={args.launch_rviz}",
        "record_bag:=true",
        "analyze:=true",
    ]

    logger.info("================================================================")
    logger.info("[setup] Pipeline '%s': planner=%s pipeline=%s", label, planner, planning_pipeline)
    logger.info("[setup] %s", " ".join(cmd))
    logger.info("================================================================")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        logger.warning("[setup] Pipeline '%s' exited with code %d; continuing to the next pipeline.",
                        label, result.returncode)
        return False
    logger.info("[setup] Pipeline '%s' finished.", label)
    return True


def main():
    parser = argparse.ArgumentParser(description="Run the FANUC benchmark across multiple planning pipelines.")
    parser.add_argument("--pipelines", default="mtc,pilz_lin,ompl",
                        help="Comma-separated pipelines to run in order (mtc | pilz_lin | ompl).")
    parser.add_argument("--output-dir", default=os.environ.get("PWD", os.getcwd()))
    parser.add_argument("--timestamp", default="auto",
                        help="Parent folder suffix ('auto' or empty -> current date-time).")
    parser.add_argument("--robot-model", default="crx30ia")
    parser.add_argument("--robot-ip", default="192.168.120.101")
    parser.add_argument("--iterations", default="3")
    parser.add_argument("--velocity-scaling", default="0.1")
    parser.add_argument("--acceleration-scaling", default="0.1")
    parser.add_argument("--planning-time", default="5.0")
    parser.add_argument("--launch-rviz", default="false")
    parser.add_argument("--log-level", default=os.environ.get("ANALYZE_BENCHMARK_LOG_LEVEL", "INFO"))
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO), format="%(levelname)s: %(message)s")

    labels = [s for s in (p.strip() for p in args.pipelines.split(",")) if s]
    if not labels:
        sys.exit("No pipelines requested (--pipelines was empty).")

    # 'auto' (the launch default) or empty -> generate a date-time stamp.
    ts = args.timestamp.strip().strip("'\"").strip()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S") if ts in ("", "auto") else ts
    parent_dir = os.path.join(args.output_dir, f"{PREFIX}_{timestamp}")
    os.makedirs(parent_dir, exist_ok=True)
    logger.info("[setup] Comparing pipelines %s -> %s", labels, parent_dir)

    ran = []
    for label in labels:
        if run_one_pipeline(label, args, parent_dir):
            ran.append(label)

    logger.info("[setup] All pipelines done (%d/%d succeeded). Aggregating comparison ...", len(ran), len(labels))
    # Aggregate every per-pipeline run that produced a CSV (auto-discovered), so a
    # pipeline that aborted mid-run is simply absent rather than blocking the rest.
    compare_benchmarks.compare(parent_dir, labels=labels, show=False)
    logger.info("[setup] Comparison written under %s", parent_dir)


if __name__ == "__main__":
    main()
