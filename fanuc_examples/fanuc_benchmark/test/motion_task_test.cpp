// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0
//
// Motion-task runner (MoveIt side).
//
// This manipulator-agnostic node executes a predefined "motion transaction"
// (MOTR): starting from Pose-begin it moves through the intermediate waypoints
// and ends at Pose-goal. The motion is repeated `iterations` times; between
// forward MOTRs the waypoints are traversed in reverse to return to Pose-begin
// (also run as a reverse MOTR). For each MOTR it measures the begin->goal
// elapsed time (split into planning and execution).
//
// The node only does work that touches the MoveIt API. Each MOTR result is
// PUBLISHED as a JSON string on /motion_task_test/motr_result, so it is captured
// by the `ros2 bag record` started in the launch file. Nothing is written to
// disk here: aggregate statistics (mean/stdev/longest/shortest), the CSV, and
// the planned-vs-actual / speed-scaling plots are all produced by
// analyze_benchmark.py, which reads the recorded rosbag.
//
// Three planner back-ends are selected with the `planner` parameter:
//   * "ompl"     - OMPL via MoveGroupInterface (pipeline "ompl")
//   * "pilz_lin" - Pilz LIN via MoveGroupInterface
//                  (pipeline "pilz_industrial_motion_planner", planner id "LIN")
//   * "mtc"      - MoveIt Task Constructor: the whole waypoint chain is planned
//                  as one task, then executed through MoveGroupInterface.

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit_task_constructor_msgs/msg/solution.hpp>

namespace mtc = moveit::task_constructor;
using moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr char kNodeName[] = "motion_task_test";
using Clock = std::chrono::steady_clock;

double secondsSince(const Clock::time_point& start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

struct Waypoint
{
  std::string name;
  std::vector<double> joints;
};

// Result of a single motion transaction.
struct MotrResult
{
  int index = 0;
  std::string direction;  // "FWD" or "REV"
  bool success = false;
  double plan_s = 0.0;
  double exec_s = 0.0;
  double total_s = 0.0;
  int segments = 0;
};

// ---------------------------------------------------------------------------
// Planner back-ends
// ---------------------------------------------------------------------------

// Common interface so the motion loop is planner-agnostic. This is also the
// extension point for future frameworks (e.g. Tesseract): implement a new
// PlannerBackend and wire it up in main().
class PlannerBackend
{
public:
  virtual ~PlannerBackend() = default;
  virtual std::string label() const = 0;

  // Untimed repositioning move to a single joint target (used to reach
  // Pose-begin before the timed run starts).
  virtual bool moveTo(const std::vector<double>& joints) = 0;

  // Plan and execute through the ordered list of targets (the current robot
  // state is the implicit start). Accumulates planning / execution time into
  // plan_s / exec_s. Returns false on the first failed segment.
  virtual bool runMotr(const std::vector<Waypoint>& targets, double& plan_s, double& exec_s) = 0;
};

// OMPL and Pilz share this back-end; they differ only in pipeline / planner id.
class MoveGroupBackend : public PlannerBackend
{
public:
  MoveGroupBackend(const std::shared_ptr<MoveGroupInterface>& move_group, std::string label,
                   const std::string& pipeline_id, const std::string& planner_id)
    : move_group_(move_group), label_(std::move(label))
  {
    if (!pipeline_id.empty())
      move_group_->setPlanningPipelineId(pipeline_id);
    if (!planner_id.empty())
      move_group_->setPlannerId(planner_id);
  }

  std::string label() const override { return label_; }

  bool moveTo(const std::vector<double>& joints) override
  {
    double plan_s = 0.0, exec_s = 0.0;
    return planAndExecute(joints, plan_s, exec_s);
  }

  bool runMotr(const std::vector<Waypoint>& targets, double& plan_s, double& exec_s) override
  {
    for (const auto& wp : targets)
    {
      if (!planAndExecute(wp.joints, plan_s, exec_s))
        return false;
    }
    return true;
  }

private:
  bool planAndExecute(const std::vector<double>& joints, double& plan_s, double& exec_s)
  {
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setJointValueTarget(joints))
    {
      RCLCPP_ERROR(rclcpp::get_logger(kNodeName), "Joint target is out of bounds");
      return false;
    }

    MoveGroupInterface::Plan plan;
    const auto plan_start = Clock::now();
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    plan_s += secondsSince(plan_start);
    if (!planned)
      return false;

    const auto exec_start = Clock::now();
    const bool executed = static_cast<bool>(move_group_->execute(plan));
    exec_s += secondsSince(exec_start);
    return executed;
  }

  std::shared_ptr<MoveGroupInterface> move_group_;
  std::string label_;
};

// MoveIt Task Constructor: plan the full waypoint chain as one task, then
// execute the resulting sub-trajectories through MoveGroupInterface (this avoids
// requiring the ExecuteTaskSolution capability in move_group).
class MtcBackend : public PlannerBackend
{
public:
  MtcBackend(rclcpp::Node::SharedPtr node, const std::shared_ptr<MoveGroupInterface>& move_group,
             std::string group, std::vector<std::string> joint_names, mtc::solvers::PlannerInterfacePtr solver)
    : node_(std::move(node))
    , move_group_(move_group)
    , group_(std::move(group))
    , joint_names_(std::move(joint_names))
    , solver_(std::move(solver))
  {
  }

  std::string label() const override { return "mtc"; }

  bool moveTo(const std::vector<double>& joints) override
  {
    double plan_s = 0.0, exec_s = 0.0;
    return runMotr({ Waypoint{ "reposition", joints } }, plan_s, exec_s);
  }

  bool runMotr(const std::vector<Waypoint>& targets, double& plan_s, double& exec_s) override
  {
    mtc::Task task;
    task.setName("motion_task");
    task.loadRobotModel(node_);
    task.setProperty("group", group_);
    task.add(std::make_unique<mtc::stages::CurrentState>("current"));

    for (const auto& wp : targets)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("to_" + wp.name, solver_);
      stage->setGroup(group_);
      std::map<std::string, double> goal;
      for (std::size_t i = 0; i < joint_names_.size(); ++i)
        goal[joint_names_[i]] = wp.joints[i];
      stage->setGoal(goal);
      task.add(std::move(stage));
    }

    const auto plan_start = Clock::now();
    const bool planned = static_cast<bool>(task.plan(1));
    plan_s += secondsSince(plan_start);
    if (!planned || task.solutions().empty())
    {
      RCLCPP_ERROR(rclcpp::get_logger(kNodeName), "MTC failed to find a solution");
      return false;
    }

    moveit_task_constructor_msgs::msg::Solution solution_msg;
    task.solutions().front()->toMsg(solution_msg, nullptr);

    const auto exec_start = Clock::now();
    for (const auto& sub : solution_msg.sub_trajectory)
    {
      if (sub.trajectory.joint_trajectory.points.empty())
        continue;
      if (!static_cast<bool>(move_group_->execute(sub.trajectory)))
      {
        exec_s += secondsSince(exec_start);
        return false;
      }
    }
    exec_s += secondsSince(exec_start);
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  std::string group_;
  std::vector<std::string> joint_names_;
  mtc::solvers::PlannerInterfacePtr solver_;
};

// Serialise a MOTR result as a one-line JSON object so analyze_benchmark.py can
// recover it from the rosbag without a custom message type.
std::string toJson(const MotrResult& r, const std::string& planner)
{
  std::ostringstream os;
  os << '{' << "\"index\":" << r.index << ",\"direction\":\"" << r.direction << "\",\"planner\":\"" << planner
     << "\",\"success\":" << (r.success ? "true" : "false") << ",\"plan_s\":" << r.plan_s
     << ",\"exec_s\":" << r.exec_s << ",\"total_s\":" << r.total_s << ",\"segments\":" << r.segments << '}';
  return os.str();
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>(kNodeName, options);
  auto logger = node->get_logger();

  // Per-MOTR results are published here and captured by `ros2 bag record`.
  // transient_local + KeepAll so every sample is retained for the recorder even
  // if it subscribes a moment after the first publish.
  auto result_pub = node->create_publisher<std_msgs::msg::String>(
      "/motion_task_test/motr_result", rclcpp::QoS(rclcpp::KeepAll()).reliable().transient_local());

  // Spin in the background so MoveGroupInterface / MTC can fetch state.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto fail = [&](const std::string& msg) {
    RCLCPP_FATAL(logger, "%s", msg.c_str());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  };

  // ---- parameters -------------------------------------------------------
  std::string planner, group, planner_id, mtc_solver_name, mtc_pipeline;
  int iterations = 0, planning_attempts = 0;
  double velocity_scaling = 0.0, acceleration_scaling = 0.0, planning_time = 0.0, state_wait_timeout = 0.0;

  node->get_parameter_or("planner", planner, std::string("ompl"));
  node->get_parameter_or("group", group, std::string("manipulator"));
  node->get_parameter_or("planner_id", planner_id, std::string(""));
  node->get_parameter_or("iterations", iterations, 3);
  node->get_parameter_or("velocity_scaling", velocity_scaling, 0.1);
  node->get_parameter_or("acceleration_scaling", acceleration_scaling, 0.1);
  node->get_parameter_or("planning_time", planning_time, 5.0);
  node->get_parameter_or("planning_attempts", planning_attempts, 10);
  node->get_parameter_or("state_wait_timeout", state_wait_timeout, 30.0);  // seconds
  node->get_parameter_or("mtc_solver", mtc_solver_name, std::string("pipeline"));
  node->get_parameter_or("mtc_pipeline", mtc_pipeline, std::string("ompl"));

  std::vector<std::string> joint_names, waypoint_names;
  node->get_parameter_or("joint_names", joint_names,
                         std::vector<std::string>{ "J1", "J2", "J3", "J4", "J5", "J6" });
  node->get_parameter_or("waypoint_names", waypoint_names, std::vector<std::string>{});

  if (waypoint_names.size() < 2)
    return fail("Need at least 2 waypoints (begin and goal). Check the poses file.");

  // Each waypoint is a per-name double array under "poses.<name>".
  std::vector<Waypoint> waypoints;
  waypoints.reserve(waypoint_names.size());
  for (const auto& name : waypoint_names)
  {
    std::vector<double> joints;
    if (!node->get_parameter("poses." + name, joints) || joints.size() != joint_names.size())
      return fail("Waypoint '" + name + "' is missing or does not have " + std::to_string(joint_names.size()) +
                  " joint values");
    waypoints.push_back(Waypoint{ name, joints });
  }

  RCLCPP_INFO(logger, "Motion task planner='%s' group='%s' iterations=%d vel=%.2f acc=%.2f", planner.c_str(),
              group.c_str(), iterations, velocity_scaling, acceleration_scaling);
  RCLCPP_INFO(logger, "Waypoints: %zu (begin='%s' goal='%s')", waypoints.size(), waypoints.front().name.c_str(),
              waypoints.back().name.c_str());

  // ---- MoveGroupInterface ----------------------------------------------
  // Bound the whole "acquire robot state" phase by a single deadline so the node
  // ABORTS (instead of hanging) when move_group or the driver is not up. The
  // constructor blocks up to wait_for_servers for the move_group action server;
  // the loop below then waits for the planning scene monitor to deliver a state.
  RCLCPP_INFO(logger, "Connecting to move_group (timeout %.1f s) ...", state_wait_timeout);
  const auto acquire_start = Clock::now();
  auto move_group = std::make_shared<MoveGroupInterface>(node, group, std::shared_ptr<tf2_ros::Buffer>(),
                                                         rclcpp::Duration::from_seconds(state_wait_timeout));
  move_group->setMaxVelocityScalingFactor(velocity_scaling);
  move_group->setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group->setPlanningTime(planning_time);
  move_group->setNumPlanningAttempts(static_cast<unsigned int>(std::max(1, planning_attempts)));

  RCLCPP_INFO(logger, "Waiting for the current robot state ...");
  bool have_state = false;
  while (rclcpp::ok() && secondsSince(acquire_start) < state_wait_timeout)
  {
    if (move_group->getCurrentState(1.0))
    {
      have_state = true;
      break;
    }
    RCLCPP_WARN_THROTTLE(logger, *node->get_clock(), 5000, "  still waiting for joint states / move_group ...");
  }
  if (!have_state)
    return fail("Timed out after " + std::to_string(state_wait_timeout) +
                " s waiting for the current robot state. Is move_group up and the driver publishing joint states?");

  // ---- build the selected back-end -------------------------------------
  std::shared_ptr<PlannerBackend> backend;
  if (planner == "ompl")
  {
    backend = std::make_shared<MoveGroupBackend>(move_group, "ompl", "ompl", planner_id);
  }
  else if (planner == "pilz_lin")
  {
    backend = std::make_shared<MoveGroupBackend>(move_group, "pilz_lin", "pilz_industrial_motion_planner",
                                                 planner_id.empty() ? "LIN" : planner_id);
  }
  else if (planner == "mtc")
  {
    mtc::solvers::PlannerInterfacePtr solver;
    if (mtc_solver_name == "interpolation")
      solver = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    else
      solver = std::make_shared<mtc::solvers::PipelinePlanner>(node, mtc_pipeline);
    solver->setProperty("max_velocity_scaling_factor", velocity_scaling);
    solver->setProperty("max_acceleration_scaling_factor", acceleration_scaling);
    solver->setTimeout(planning_time);
    backend = std::make_shared<MtcBackend>(node, move_group, group, joint_names, solver);
  }
  else
  {
    return fail("Unknown planner '" + planner + "' (expected ompl | pilz_lin | mtc)");
  }

  // Forward MOTR target list (begin is the implicit start) and its reverse.
  std::vector<Waypoint> forward_targets(waypoints.begin() + 1, waypoints.end());
  std::vector<Waypoint> reverse_targets(waypoints.rbegin() + 1, waypoints.rend());

  // Move to Pose-begin before timing anything.
  RCLCPP_INFO(logger, "Moving to start pose '%s' ...", waypoints.front().name.c_str());
  if (!backend->moveTo(waypoints.front().joints))
    return fail("Failed to reach the start pose; aborting.");

  // ---- main motion loop ------------------------------------------------
  std::size_t completed = 0;
  int motr_index = 0;
  for (int it = 0; it < iterations && rclcpp::ok(); ++it)
  {
    for (const bool forward : { true, false })
    {
      const auto& targets = forward ? forward_targets : reverse_targets;
      MotrResult r;
      r.index = ++motr_index;
      r.direction = forward ? "FWD" : "REV";
      r.segments = static_cast<int>(targets.size());

      RCLCPP_INFO(logger, "MOTR %d [%s] iteration %d/%d ...", r.index, r.direction.c_str(), it + 1, iterations);
      const auto motr_start = Clock::now();
      r.success = backend->runMotr(targets, r.plan_s, r.exec_s);
      r.total_s = secondsSince(motr_start);

      // Publish the raw measurement (recorded into the rosbag).
      std_msgs::msg::String msg;
      msg.data = toJson(r, backend->label());
      result_pub->publish(msg);
      ++completed;

      if (!r.success)
        RCLCPP_ERROR(logger, "MOTR %d [%s] FAILED after %.3f s", r.index, r.direction.c_str(), r.total_s);
      else
        RCLCPP_INFO(logger, "MOTR %d [%s] done: total=%.3f s (plan=%.3f exec=%.3f)", r.index, r.direction.c_str(),
                    r.total_s, r.plan_s, r.exec_s);
    }
  }

  RCLCPP_INFO(logger, "Motion task complete: %zu MOTRs published on /motion_task_test/motr_result.", completed);
  RCLCPP_INFO(logger, "Run analyze_benchmark.py on the recorded bag for stats and plots.");

  // Give the recorder a moment to capture the final messages before exiting.
  std::this_thread::sleep_for(std::chrono::seconds(2));

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
