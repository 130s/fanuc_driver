<!--
SPDX-FileCopyrightText: 2026, FANUC America Corporation
SPDX-FileCopyrightText: 2026, FANUC CORPORATION
SPDX-License-Identifier: Apache-2.0
-->
# fanuc_benchmark

Motion-planner benchmark for FANUC CRX robots. It drives a defined **motion
transaction (MOTR)** — from Pose-begin, through the intermediate waypoints, to
Pose-goal — repeatedly through MoveIt, and compares planners by their begin→goal
timing while recording planned-vs-actual joint trajectories and the
collaborative speed scaling.

Supported planners (selected with the `planner` launch argument):

| `planner`  | How it plans |
|------------|--------------|
| `ompl`     | OMPL via `MoveGroupInterface` (pipeline `ompl`) |
| `pilz_lin` | Pilz **LIN** via `MoveGroupInterface` (pipeline `pilz_industrial_motion_planner`, planner id `LIN`) |
| `mtc`      | **MoveIt Task Constructor**: the whole waypoint chain is planned as one task, then executed through `MoveGroupInterface` |

The design is split so it is easy to extend (e.g. add a Tesseract back-end):

- **`test/motion_task_test.cpp`** — the only code that touches the MoveIt API.
  A manipulator-agnostic node that runs the MOTRs, times each one
  (planning + execution), and **publishes** each result as JSON on
  `/motion_task_test/motr_result`. To add a framework, implement a new
  `PlannerBackend` and wire it up in `main()`.
- **`test/analyze_benchmark.py`** — no MoveIt. Reads the recorded rosbag and
  prints/writes the timing statistics and the plots.
- **`test/benchmark.launch.xml`** — orchestration (move_group + driver, rosbag
  recording, the node). The one piece that XML cannot express — building the
  MoveIt config — lives in the small helper **`test/motion_task_test.launch.py`**.

## Waypoints

Edit `test/benchmark_poses.yaml`. The first entry in `waypoint_names` is
Pose-begin, the last is Pose-goal, the rest are intermediate waypoints. A
forward MOTR runs them top-to-bottom; the reverse MOTR runs them bottom-to-top.

## Build

```bash
# MoveIt Task Constructor is required by this package:
sudo apt install ros-$ROS_DISTRO-moveit-task-constructor-core \
                 ros-$ROS_DISTRO-moveit-task-constructor-msgs
# (or: rosdep install --from-paths <ws>/src --ignore-src -y)

colcon build --packages-select fanuc_benchmark
source install/setup.bash
```

## Run

One command brings up the driver + move_group, records a rosbag, and runs the
benchmark. **Real hardware only** — set `robot_ip`.

```bash
# OMPL (default)
ros2 launch fanuc_benchmark benchmark.launch.xml robot_ip:=192.168.120.101

# Pilz LIN (needs the pilz pipeline + pilz joint limits)
ros2 launch fanuc_benchmark benchmark.launch.xml \
  planner:=pilz_lin \
  planning_pipeline:=pilz_industrial_motion_planner \
  joint_limits_file:=<abs path>/fanuc_moveit_config/config/pilz_joint_limits.yaml

# MoveIt Task Constructor
ros2 launch fanuc_benchmark benchmark.launch.xml planner:=mtc
```

If `move_group` is already running (e.g. via `fanuc_moveit.launch.py`), add
`launch_moveit:=false` to run only the node + recording.

Useful arguments: `iterations` (default 3), `velocity_scaling` /
`acceleration_scaling` (default 0.1 — raise carefully), `planning_time`,
`mtc_solver` (`pipeline` | `interpolation`), `record_bag`, `output_dir`.

The bag is written to `output_dir` (default: current directory) as
`benchmark_fanuc-driver_<TIMESTAMP>`. Recorded topics: `/motion_task_test/motr_result`,
`/joint_states`, `/joint_trajectory_controller/controller_state` (planned =
`reference`, actual = `feedback`), `/fanuc_gpio_controller/collaborative_speed_scaling`,
`/display_planned_path`.

## Analyze

```bash
ros2 run fanuc_benchmark analyze_benchmark.py <output_dir>/benchmark_fanuc-driver_<TIMESTAMP>
# or just the newest benchmark_* bag in the current directory:
ros2 run fanuc_benchmark analyze_benchmark.py
```

This prints the per-MOTR begin→goal times and the aggregate statistics
(count / mean / stdev / longest / shortest), the collaborative
speed-scaling average / max / min / stdev, and the number of
`fanuc_msgs/srv/SwitchControlState` (STMO recovery) service calls, and writes
(all sharing the `benchmark_fanuc-driver` base name):

| File | Contents |
|------|----------|
| `benchmark_fanuc-driver.csv` | per-MOTR timings |
| `benchmark_fanuc-driver_speed_scaling_stats.csv` | `/fanuc_gpio_controller/collaborative_speed_scaling` average / max / min / stdev |
| `benchmark_fanuc-driver_stmo_service_calls.csv` | count of `SwitchControlState` STMO-recovery service calls |
| `benchmark_fanuc-driver_joint_states.csv` | time-lapsed J1..J6 **actual** positions from `/joint_states` |
| `benchmark_fanuc-driver_planned_path.csv` | time-lapsed J1..J6 **planned** positions from `/display_planned_path` (the motion planner's plan) |
| `benchmark_fanuc-driver_joints.png` | planned vs. actual per joint |
| `benchmark_fanuc-driver_speed_scaling.png` | speed scaling over time |
| `benchmark_fanuc-driver_overview.png` | **single combined image**: speed scaling (avg / max / min / ±stdev) + J1..J6 actual (`/joint_states`) vs planner plan (`/display_planned_path`), shared time axis |

The two joint-position CSVs share the bag's `t=0`, so the actual trace and the
planner's plan line up on the same time axis for later plotting. Use `--no-show`
for headless runs and `--log-level DEBUG|INFO|WARNING` to control verbosity.

## Compare multiple pipelines

To benchmark several planners in one go, use `benchmark_setup.launch.xml`. It
runs the benchmark **once per pipeline** (each a clean `benchmark.launch.xml`
relaunch, so every pipeline gets its correct `move_group` config — Pilz its own
joint-limits file — and the driver is brought up fresh; each run moves the arm
to the start pose first, which doubles as the "return to start between
pipelines" step):

```bash
# all three pipelines (mtc, pilz_lin, ompl), in order
ros2 launch fanuc_benchmark benchmark_setup.launch.xml robot_ip:=192.168.120.101

# a subset, with more iterations
ros2 launch fanuc_benchmark benchmark_setup.launch.xml \
  pipelines:=ompl,pilz_lin iterations:=5
```

Everything lands in one parent folder, grouped by pipeline:

```text
benchmark_fanuc-driver_<TIMESTAMP>/
  benchmark_fanuc-driver_mtc/        # bag + per-run CSVs/PNGs (analyze_benchmark.py)
  benchmark_fanuc-driver_pilz_lin/
  benchmark_fanuc-driver_ompl/
  benchmark_fanuc-driver_comparison.csv        # every MOTR, tagged with its pipeline
  benchmark_fanuc-driver_comparison_stats.csv  # per-pipeline mean/stdev/min/max begin→goal
  benchmark_fanuc-driver_comparison.png        # mean begin→goal per pipeline (FWD/REV/All)
```

The orchestration (`run_pipeline_comparison.py`) and the aggregation
(`compare_benchmarks.py`) are separate scripts; you can re-aggregate an existing
parent folder at any time with:

```bash
ros2 run fanuc_benchmark compare_benchmarks.py <parent_folder>
```

## Notes

- Velocity/acceleration scaling default to **0.1** for safety on real hardware.
- A MOTR plans and executes one waypoint segment at a time (so the arm pauses at
  intermediate waypoints) for `ompl`/`pilz_lin`; for `mtc` the whole chain is
  planned together and executed segment by segment. The begin→goal elapsed time
  is wall-clock over the full transaction.
- The `mtc` back-end executes solutions through `MoveGroupInterface`, so it does
  **not** require the `ExecuteTaskSolution` capability in `move_group`.
