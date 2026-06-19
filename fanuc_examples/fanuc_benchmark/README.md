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
`benchmark_<TIMESTAMP>`. Recorded topics: `/motion_task_test/motr_result`,
`/joint_states`, `/joint_trajectory_controller/controller_state` (planned =
`reference`, actual = `feedback`), `/fanuc_gpio_controller/collaborative_speed_scaling`,
`/display_planned_path`.

## Analyze

```bash
ros2 run fanuc_benchmark analyze_benchmark.py <output_dir>/benchmark_<TIMESTAMP>
# or just the newest benchmark_* bag in the current directory:
ros2 run fanuc_benchmark analyze_benchmark.py
```

This prints the per-MOTR begin→goal times and the aggregate statistics
(count / mean / stdev / longest / shortest), writes `benchmark_timings.csv`, and
saves `benchmark_joints.png` (planned vs. actual per joint) and
`benchmark_speed_scaling.png`. Use `--no-show` for headless runs and
`--log-level DEBUG|INFO|WARNING` to control verbosity.

## Notes

- Velocity/acceleration scaling default to **0.1** for safety on real hardware.
- A MOTR plans and executes one waypoint segment at a time (so the arm pauses at
  intermediate waypoints) for `ompl`/`pilz_lin`; for `mtc` the whole chain is
  planned together and executed segment by segment. The begin→goal elapsed time
  is wall-clock over the full transaction.
- The `mtc` back-end executes solutions through `MoveGroupInterface`, so it does
  **not** require the `ExecuteTaskSolution` capability in `move_group`.
