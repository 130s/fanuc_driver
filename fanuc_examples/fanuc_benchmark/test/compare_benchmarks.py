#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0
#
# Aggregate several single-pipeline benchmark runs into ONE labeled comparison.
#
# benchmark_setup.launch.xml runs the benchmark once per planning pipeline (MTC,
# Pilz LIN, OMPL), each as a clean relaunch that drops its results into a
# per-pipeline subfolder of a single parent run folder:
#
#   <parent>/benchmark_fanuc-driver_<pipeline>/benchmark_fanuc-driver.csv
#
# This script reads each of those per-MOTR CSVs, tags every row with its
# pipeline, and writes the headline deliverables at the parent level:
#
#   <parent>/benchmark_fanuc-driver_comparison.csv       (per-MOTR, pipeline-tagged)
#   <parent>/benchmark_fanuc-driver_comparison_stats.csv (per-pipeline aggregates)
#   <parent>/benchmark_fanuc-driver_comparison.png       (mean begin->goal per pipeline)
#
# Usage:
#   ros2 run fanuc_benchmark compare_benchmarks.py <parent_dir> [--labels a,b,c]
#   # or: python3 compare_benchmarks.py <parent_dir>
#
# With no --labels, every benchmark_fanuc-driver_*/benchmark_fanuc-driver.csv
# under <parent_dir> is discovered automatically (the label is the subfolder name
# minus the benchmark_fanuc-driver_ prefix).

import argparse
import csv
import glob
import logging
import os
import statistics
import sys

logger = logging.getLogger("compare_benchmarks")

PREFIX = "benchmark_fanuc-driver"
PER_RUN_CSV = f"{PREFIX}.csv"


def discover_runs(parent_dir, labels):
    """Return [(label, csv_path), ...] for each per-pipeline run under parent_dir."""
    runs = []
    if labels:
        for label in labels:
            csv_path = os.path.join(parent_dir, f"{PREFIX}_{label}", PER_RUN_CSV)
            runs.append((label, csv_path))
        return runs
    # Auto-discover: <parent>/benchmark_fanuc-driver_<label>/benchmark_fanuc-driver.csv
    for csv_path in sorted(glob.glob(os.path.join(parent_dir, f"{PREFIX}_*", PER_RUN_CSV))):
        subdir = os.path.basename(os.path.dirname(csv_path))
        label = subdir[len(PREFIX) + 1:] if subdir.startswith(PREFIX + "_") else subdir
        runs.append((label, csv_path))
    return runs


def read_run(csv_path):
    """Read one per-MOTR CSV into a list of dict rows (or [] if missing)."""
    if not os.path.exists(csv_path):
        logger.warning("Missing per-pipeline CSV: %s (pipeline run skipped or failed?)", csv_path)
        return []
    with open(csv_path, newline="") as f:
        return list(csv.DictReader(f))


def _f(row, key):
    try:
        return float(row.get(key, "") or "nan")
    except ValueError:
        return float("nan")


def pipeline_stats(rows):
    """Aggregate begin->goal total_s over SUCCESSFUL MOTRs, split by direction."""
    def totals(direction):
        return [
            _f(r, "total_s")
            for r in rows
            if str(r.get("success")) == "1" and (direction is None or r.get("direction") == direction)
        ]

    def agg(values):
        values = [v for v in values if v == v]  # drop NaN
        if not values:
            return dict(count=0, mean=float("nan"), stdev=float("nan"), min=float("nan"), max=float("nan"))
        return dict(
            count=len(values),
            mean=statistics.fmean(values),
            stdev=statistics.stdev(values) if len(values) > 1 else 0.0,
            min=min(values),
            max=max(values),
        )

    return {"FWD": agg(totals("FWD")), "REV": agg(totals("REV")), "All": agg(totals(None))}


def write_comparison_csv(out_dir, runs_rows):
    path = os.path.join(out_dir, f"{PREFIX}_comparison.csv")
    cols = ["pipeline", "motr_index", "direction", "planner", "success", "plan_s", "exec_s", "total_s", "segments"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for label, rows in runs_rows:
            for r in rows:
                w.writerow([label] + [r.get(c, "") for c in cols[1:]])
    logger.info("Wrote pipeline-tagged per-MOTR comparison to %s", path)
    return path


def write_stats_csv(out_dir, stats_by_label):
    path = os.path.join(out_dir, f"{PREFIX}_comparison_stats.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pipeline", "direction", "count", "mean_total_s", "stdev_total_s", "min_total_s", "max_total_s"])
        for label, stats in stats_by_label:
            for direction in ("FWD", "REV", "All"):
                s = stats[direction]
                w.writerow([label, direction, s["count"], s["mean"], s["stdev"], s["min"], s["max"]])
    logger.info("Wrote per-pipeline comparison stats to %s", path)
    return path


def log_stats_table(stats_by_label):
    logger.info("================ PIPELINE COMPARISON (begin->goal total_s) ================")
    logger.info("  %-10s %-4s  %5s  %9s  %9s  %9s  %9s", "pipeline", "dir", "n", "mean", "stdev", "min", "max")
    for label, stats in stats_by_label:
        for direction in ("FWD", "REV", "All"):
            s = stats[direction]
            if s["count"] == 0:
                logger.info("  %-10s %-4s  %5d  %9s  %9s  %9s  %9s", label, direction, 0, "-", "-", "-", "-")
            else:
                logger.info(
                    "  %-10s %-4s  %5d  %9.3f  %9.3f  %9.3f  %9.3f",
                    label, direction, s["count"], s["mean"], s["stdev"], s["min"], s["max"],
                )
    logger.info("==========================================================================")


def plot_comparison(out_dir, stats_by_label, show):
    try:
        import matplotlib
        if not show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available; skipping comparison plot.")
        return None

    labels = [label for label, _ in stats_by_label]
    directions = ["FWD", "REV", "All"]
    x = range(len(labels))
    width = 0.25

    fig, ax = plt.subplots(figsize=(max(6, 1.8 * len(labels) + 3), 4.5))
    for di, direction in enumerate(directions):
        means = [stats[direction]["mean"] for _, stats in stats_by_label]
        errs = [stats[direction]["stdev"] for _, stats in stats_by_label]
        means = [0.0 if m != m else m for m in means]
        errs = [0.0 if e != e else e for e in errs]
        offsets = [xi + (di - 1) * width for xi in x]
        ax.bar(offsets, means, width, yerr=errs, capsize=3, label=direction)

    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.set_ylabel("mean begin->goal total [s]")
    ax.set_title("Motion-planner pipeline comparison (mean ± stdev over successful MOTRs)")
    ax.legend(title="direction")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    path = os.path.join(out_dir, f"{PREFIX}_comparison.png")
    fig.savefig(path, dpi=120)
    logger.info("Saved pipeline comparison plot %s", path)
    if show:
        plt.show()
    return path


def compare(parent_dir, labels=None, show=False):
    runs = discover_runs(parent_dir, labels)
    if not runs:
        logger.warning("No per-pipeline runs found under %s; nothing to compare.", parent_dir)
        return

    runs_rows = [(label, read_run(csv_path)) for label, csv_path in runs]
    present = [(label, rows) for label, rows in runs_rows if rows]
    if not present:
        logger.warning("All per-pipeline CSVs were missing/empty under %s.", parent_dir)
        return

    stats_by_label = [(label, pipeline_stats(rows)) for label, rows in present]

    write_comparison_csv(parent_dir, present)
    write_stats_csv(parent_dir, stats_by_label)
    log_stats_table(stats_by_label)
    plot_comparison(parent_dir, stats_by_label, show)


def main():
    parser = argparse.ArgumentParser(description="Compare per-pipeline FANUC benchmark runs into one labeled report.")
    parser.add_argument("parent_dir", help="Parent run folder containing the per-pipeline subfolders.")
    parser.add_argument("--labels", help="Comma-separated pipeline labels (default: auto-discover).")
    parser.add_argument("--show", action="store_true", help="Open the comparison plot window.")
    parser.add_argument("--log-level", default=os.environ.get("ANALYZE_BENCHMARK_LOG_LEVEL", "INFO"))
    args = parser.parse_args()

    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO), format="%(levelname)s: %(message)s")

    if not os.path.isdir(args.parent_dir):
        sys.exit(f"Parent dir '{args.parent_dir}' is not a directory.")
    labels = [s for s in (args.labels.split(",") if args.labels else []) if s]
    compare(args.parent_dir, labels=labels or None, show=args.show)


if __name__ == "__main__":
    main()
