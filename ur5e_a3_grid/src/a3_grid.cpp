// SPDX-License-Identifier: BSD-3-Clause
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
constexpr double PI = 3.14159265358979323846;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using Transform = Eigen::Isometry3d;

template <class T>
T param(const rclcpp::Node::SharedPtr& node, const std::string& name, T fallback)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, fallback);
  return node->get_parameter(name).get_value<T>();
}

Eigen::Vector3d vector3(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  const auto v = param<std::vector<double>>(node, name, {0.0, 0.0, 0.0});
  if (v.size() != 3 || !std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); }))
    throw std::runtime_error(name + " requires three finite numbers");
  return {v[0], v[1], v[2]};
}

geometry_msgs::msg::Pose pose(const Transform& t)
{
  geometry_msgs::msg::Pose p;
  p.position.x = t.translation().x();
  p.position.y = t.translation().y();
  p.position.z = t.translation().z();
  const Eigen::Quaterniond q(t.rotation());
  p.orientation.x = q.x(); p.orientation.y = q.y();
  p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}

geometry_msgs::msg::Point point(const Eigen::Vector3d& p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x(); out.y = p.y(); out.z = p.z();
  return out;
}

struct Cell
{
  int id, row, column;
  Transform tcp;
};

struct Motion
{
  std::string label;
  bool gripper = false;
  Transform tcp = Transform::Identity();
  double joint = 0.0;
};

class Grid
{
public:
  explicit Grid(const rclcpp::Node::SharedPtr& node) : node_(node), logger_(node->get_logger())
  {
    group_ = param<std::string>(node, "planning_group", "ur5e_arm");
    home_name_ = param<std::string>(node, "home_pose", "home");
    frame_ = param<std::string>(node, "reference_frame", "world");
    tip_ = param<std::string>(node, "ik_link", "tool0");
    tcp_ = param<std::string>(node, "tcp_link", "grasp_link");
    approach_link_ = param<std::string>(node, "approach_link", "ag95_base_link");
    orientation_mode_ = param<std::string>(node, "orientation_mode", "vertical");
    if (orientation_mode_ != "vertical" && orientation_mode_ != "home")
      throw std::runtime_error("orientation_mode must be vertical or home");
    operation_ = param<std::string>(node, "operation", "scan");
    execute_ = param<bool>(node, "execute", false) && operation_ != "preview";
    size_x_ = param<double>(node, "size_x", 0.297);
    size_y_ = param<double>(node, "size_y", 0.420);
    columns_ = param<int>(node, "columns", 4);
    rows_ = param<int>(node, "rows", 6);
    step_ = param<double>(node, "cartesian_step", 0.005);
    jump_ = param<double>(node, "jump_threshold", 5.0);
    max_joint_step_ = param<double>(node, "max_joint_step", 0.35);
    height_tolerance_ = param<double>(node, "height_tolerance", 0.001);
    angle_tolerance_ = param<double>(node, "orientation_tolerance", 0.001);
    velocity_ = param<double>(node, "velocity_scaling", 0.2);
    acceleration_ = param<double>(node, "acceleration_scaling", 0.2);
    hand_group_ = param<std::string>(node, "gripper_group", "dh_ag95_gripper");
    hand_joint_ = param<std::string>(node, "gripper_joint", "left_outer_knuckle_joint");
    table_link_ = param<std::string>(node, "table_link", "workbench");
    table_z_ = param<double>(node, "table_surface_z", 0.8);
    param<std::vector<double>>(node, "block_size_xyz", {0.03, 0.03, 0.03});
    block_size_ = vector3(node, "block_size_xyz");
    grasp_height_ = param<double>(node, "grasp_height_above_table", 0.015);
    hand_open_ = param<double>(node, "gripper_open_position", 0.4);
    hand_closed_ = param<double>(node, "gripper_grasp_position", 0.67);
    hand_velocity_ = param<double>(node, "gripper_velocity_scaling", 0.4);
    hand_acceleration_ = param<double>(node, "gripper_acceleration_scaling", 0.4);
    hand_pause_ = param<double>(node, "gripper_pause", 0.2);
    for (double v : {size_x_, size_y_, step_, jump_, max_joint_step_, height_tolerance_,
                     angle_tolerance_, velocity_, acceleration_})
      if (!std::isfinite(v) || v <= 0.0)
        throw std::runtime_error("Grid sizes, tolerances, step, jump threshold and scaling must be positive");
    if (columns_ < 1 || rows_ < 1 || columns_ > 100 || rows_ > 100 ||
        velocity_ > 1.0 || acceleration_ > 1.0)
      throw std::runtime_error("Invalid grid count or scaling > 1.0");
    if (operation_ != "scan" && operation_ != "cell" && operation_ != "home" && operation_ != "preview" &&
        operation_ != "pick_cycle" && operation_ != "pick_cell")
      throw std::runtime_error("operation must be scan, cell, home, preview, pick_cycle, or pick_cell");
    if (!std::isfinite(table_z_) || !std::isfinite(grasp_height_) || !std::isfinite(hand_open_) ||
        !std::isfinite(hand_closed_) || !std::isfinite(hand_pause_) || hand_pause_ < 0.0 ||
        !std::isfinite(hand_velocity_) || hand_velocity_ <= 0.0 || hand_velocity_ > 1.0 ||
        !std::isfinite(hand_acceleration_) || hand_acceleration_ <= 0.0 || hand_acceleration_ > 1.0)
      throw std::runtime_error("Invalid table/gripper parameters");
    if (isPick() && ((block_size_.array() <= 0.0).any() || grasp_height_ <= 0.0 ||
                     grasp_height_ >= block_size_.z() || hand_closed_ <= hand_open_))
      throw std::runtime_error("Pick needs positive block dimensions, a grasp height inside the block, and grasp position > open");
    const double yaw = param<double>(node, "grid_yaw_deg", 0.0);
    if (!std::isfinite(yaw))
      throw std::runtime_error("grid_yaw_deg must be finite");
    grid_rotation_ = Eigen::AngleAxisd(yaw * PI / 180.0, Eigen::Vector3d::UnitZ());
    markers_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/ur5e_a3_grid/markers", rclcpp::QoS(1).transient_local());
    display_ = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "/display_planned_path", rclcpp::QoS(1).transient_local());
  }

  bool run()
  {
    MoveGroup move(node_, group_, nullptr, rclcpp::Duration::from_seconds(20.0));
    model_ = move.getRobotModel();
    joint_group_ = model_->getJointModelGroup(group_);
    if (!joint_group_ || !model_->getLinkModel(frame_) || !model_->getLinkModel(tip_) ||
        !model_->getLinkModel(tcp_))
      throw std::runtime_error("Planning group/frame/IK link/TCP is absent from the loaded model");
    move.setPoseReferenceFrame(frame_);
    if (!move.setEndEffectorLink(tip_))
      throw std::runtime_error("MoveIt rejected the configured IK link");
    move.setMaxVelocityScalingFactor(velocity_);
    move.setMaxAccelerationScalingFactor(acceleration_);
    move.setPlanningTime(param<double>(node_, "planning_time", 10.0));
    move.setNumPlanningAttempts(5);

    auto measured = move.getCurrentState(10.0);
    if (!measured)
      throw std::runtime_error("No joint_states; start the matching MoveIt demo in the same ROS domain");
    moveit::core::RobotState home(*measured);
    if (!home.setToDefaultValues(joint_group_, home_name_))
      throw std::runtime_error("Named arm pose not found: " + home_name_);
    home.update();
    const auto world_from_frame = home.getGlobalLinkTransform(frame_);
    const Transform home_tcp = world_from_frame.inverse() * home.getGlobalLinkTransform(tcp_);
    tip_from_tcp_ = home.getGlobalLinkTransform(tip_).inverse() * home.getGlobalLinkTransform(tcp_);
    center_ = home_tcp;
    if (!param<bool>(node_, "center_from_home", true))
      center_.translation() = vector3(node_, "center_xyz");
    center_.translation() += vector3(node_, "center_offset_xyz");
    setOrientation(home);
    // Keep the original named home's position, but solve a new working-home
    // state for the selected gripper orientation. The SRDF is not changed.
    moveit::core::RobotState working_home(home);
    auto scene = readScene();
    const moveit::core::GroupStateValidityCallbackFn valid =
        [scene](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
                const double* values) {
          state->setJointGroupPositions(group, values);
          state->update();
          return state->satisfiesBounds(group) && !scene->isStateColliding(*state, "", false);
        };
    const Transform home_tip_goal = world_from_frame * center_ * tip_from_tcp_.inverse();
    if (!working_home.setFromIK(joint_group_, home_tip_goal, tip_, 1.0, valid))
      throw std::runtime_error("Working home is unreachable at the selected TCP position/orientation; nothing executed");
    working_home.update();
    if (!onPlane(working_home))
      throw std::runtime_error("Working-home IK failed the height/orientation tolerance");
    createCells();
    exportCoordinates();
    publishGrid();
    timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() { publishGrid(); });
    RCLCPP_INFO(logger_, "A3 centre TCP [%s] XYZ=(%.6f, %.6f, %.6f) m; %d x %d cells; pitch=(%.3f, %.3f) mm",
                frame_.c_str(), center_.translation().x(), center_.translation().y(),
                center_.translation().z(), columns_, rows_, 1000.0 * size_x_ / columns_,
                1000.0 * size_y_ / rows_);

    if (operation_ == "home")
      return goHome(move, *measured, working_home);

    if (isPick())
      return pickCycle(move, *measured, working_home);

    // Check the full A3 boundary too, not just the inset cell centres.
    if (param<bool>(node_, "check_corners", true))
    {
      for (const auto& xy : std::vector<Eigen::Vector2d>{{-0.5, -0.5}, {-0.5, 0.5}, {0.5, 0.5}, {0.5, -0.5}})
      {
        const auto corner = target(xy.x() * size_x_, xy.y() * size_y_);
        moveit_msgs::msg::RobotTrajectory checked;
        if (!cartesian(move, working_home, {corner}, checked, "A3 corner"))
          throw std::runtime_error("A3 boundary is unreachable at the requested height/orientation; nothing executed");
      }
      RCLCPP_INFO(logger_, "All four A3 corners passed collision-aware Cartesian planning from home");
    }

    std::vector<Transform> goals;
    if (operation_ == "cell")
    {
      const int id = param<int>(node_, "cell_id", 1);
      if (id < 1 || id > static_cast<int>(cells_.size()))
        throw std::runtime_error("cell_id is outside the configured grid");
      goals.push_back(cells_[static_cast<std::size_t>(id - 1)].tcp);
      RCLCPP_INFO(logger_, "Selected cell %d", id);
    }
    else
    {
      for (int row = 0; row < rows_; ++row)
        for (int column = 0; column < columns_; ++column)
        {
          const int c = row % 2 == 0 ? column : columns_ - 1 - column;
          goals.push_back(cells_[static_cast<std::size_t>(row * columns_ + c)].tcp);
        }
      if (param<bool>(node_, "return_to_center", true))
        goals.push_back(center_);
    }

    const bool prepare_home = !onPlane(*measured);
    if (prepare_home && !param<bool>(node_, "prepare_home_if_needed", true))
      throw std::runtime_error("Current TCP is outside the configured height/orientation; move to home first");
    const moveit::core::RobotState& start = prepare_home ? working_home : *measured;
    moveit_msgs::msg::RobotTrajectory trajectory;
    if (!cartesian(move, start, goals, trajectory, "grid route"))
      throw std::runtime_error("The full grid route failed preflight; no partial route is executed");
    retimeAndValidate(start, trajectory);
    showTrajectory(start, trajectory);
    if (!execute_)
    {
      RCLCPP_INFO(logger_, "PLAN ONLY: %zu Cartesian goals passed. execute:=true enables execution. Home preparation needed=%s",
                  goals.size(), prepare_home ? "yes" : "no");
      return true;
    }
    if (prepare_home)
    {
      RCLCPP_INFO(logger_, "Preparing the working home first; fixed-height grid motion starts after preparation");
      if (!goHome(move, *measured, working_home))
        return false;
    }
    // Refresh and replan from measured feedback, including after home preparation.
    measured = move.getCurrentState(5.0);
    if (!measured || !onPlane(*measured))
      throw std::runtime_error("Measured TCP has not reached the configured height/orientation");
    if (!cartesian(move, *measured, goals, trajectory, "measured-start grid route"))
      return false;
    retimeAndValidate(*measured, trajectory);
    showTrajectory(*measured, trajectory);
    MoveGroup::Plan plan;
    moveit::core::robotStateToRobotStateMsg(*measured, plan.start_state_);
    plan.trajectory_ = trajectory;
    RCLCPP_INFO(logger_, "Executing %zu goals at fixed TCP height %.6f m [%s]",
                goals.size(), center_.translation().z(), frame_.c_str());
    const auto result = move.execute(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Grid execution failed: MoveIt code=" + std::to_string(result.val));
    measured = move.getCurrentState(5.0);
    if (!measured)
      throw std::runtime_error("No joint feedback after execution");
    const Transform actual = measured->getGlobalLinkTransform(frame_).inverse() * measured->getGlobalLinkTransform(tcp_);
    const double position_error = (actual.translation() - goals.back().translation()).norm();
    if (!onPlane(*measured) || position_error > 0.003)
      throw std::runtime_error("Final measured TCP is outside the endpoint tolerance");
    RCLCPP_INFO(logger_, "SUCCESS: %s complete; final TCP=(%.6f, %.6f, %.6f) m; endpoint error=%.3f mm",
                operation_.c_str(), actual.translation().x(), actual.translation().y(),
                actual.translation().z(), 1000.0 * position_error);
    return true;
  }

private:
  bool isPick() const { return operation_ == "pick_cycle" || operation_ == "pick_cell"; }

  void setOrientation(const moveit::core::RobotState& home)
  {
    if (!model_->getLinkModel(approach_link_) || !model_->getLinkModel(table_link_))
      throw std::runtime_error("approach_link/table_link is absent from the robot model");
    const Transform frame_from_world = home.getGlobalLinkTransform(frame_).inverse();
    const Transform approach = frame_from_world * home.getGlobalLinkTransform(approach_link_);
    const Transform table = frame_from_world * home.getGlobalLinkTransform(table_link_);
    const Eigen::Vector3d down = -table.linear().col(2);
    // Grid moves use reference-frame XY/Z, so its Z must be the table normal.
    if (down.dot(-Eigen::Vector3d::UnitZ()) < 1.0 - 1e-8)
      throw std::runtime_error("reference_frame Z must point along the upward table normal");
    const double old_tilt = std::acos(std::clamp(approach.linear().col(2).dot(down), -1.0, 1.0));
    if (orientation_mode_ == "vertical")
    {
      // Preserve the gripper X heading on the table, make its approach Z point
      // straight down, then include the fixed approach-link -> TCP rotation.
      Eigen::Vector3d x = approach.linear().col(0);
      x -= down * down.dot(x);
      if (x.norm() < 1e-8)
        throw std::runtime_error("Home gripper X heading cannot be projected onto the table");
      x.normalize();
      Eigen::Matrix3d upright;
      upright.col(0) = x;
      upright.col(1) = down.cross(x).normalized();
      upright.col(2) = down;
      center_.linear() = upright * approach.linear().transpose() * center_.linear();
    }
    const Eigen::Matrix3d desired_approach = center_.linear() *
        (home.getGlobalLinkTransform(tcp_).linear().transpose() * home.getGlobalLinkTransform(approach_link_).linear());
    const double target_tilt = std::acos(std::clamp(desired_approach.col(2).dot(down), -1.0, 1.0));
    RCLCPP_INFO(logger_, "Gripper orientation=%s: named-home tilt=%.6f deg, target tilt=%.6f deg from downward table normal",
                orientation_mode_.c_str(), old_tilt * 180.0 / PI, target_tilt * 180.0 / PI);
  }

  std::vector<int> visitOrder() const
  {
    if (operation_ == "pick_cell")
    {
      const int id = param<int>(node_, "cell_id", 1);
      if (id < 1 || id > static_cast<int>(cells_.size()))
        throw std::runtime_error("cell_id is outside the configured grid");
      return {id};
    }
    std::vector<int> result;
    for (int row = 0; row < rows_; ++row)
      for (int column = 0; column < columns_; ++column)
        result.push_back(row * columns_ + (row % 2 == 0 ? column : columns_ - 1 - column) + 1);
    return result;
  }

  planning_scene::PlanningScenePtr readScene()
  {
    if (!scene_client_)
      scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
          param<std::string>(node_, "planning_scene_service", "/get_planning_scene"));
    if (!scene_client_->wait_for_service(std::chrono::seconds(10)))
      throw std::runtime_error("MoveIt planning scene service is unavailable");
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    using Components = moveit_msgs::msg::PlanningSceneComponents;
    request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
        Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_GEOMETRY |
        Components::OCTOMAP | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX |
        Components::LINK_PADDING_AND_SCALING;
    auto future = scene_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
      throw std::runtime_error("Timed out reading MoveIt planning scene");
    auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
    scene->setPlanningSceneMsg(future.get()->scene);
    if (isPick())
    {
      if (!model_->getLinkModel(table_link_))
        throw std::runtime_error("table_link does not exist in the robot model");
      // Restore table checks for all moving descendants, including the gripper.
      // Keep the fixed base/table mounting contact and the user's global SRDF unchanged.
      scene->getAllowedCollisionMatrixNonConst().setEntry(
          table_link_, joint_group_->getUpdatedLinkModelNames(), false);
    }
    return scene;
  }

  moveit::core::RobotState finalState(const moveit::core::RobotState& start,
                                    const moveit_msgs::msg::RobotTrajectory& message,
                                    const std::string& group) const
  {
    robot_trajectory::RobotTrajectory trajectory(model_, group);
    trajectory.setRobotTrajectoryMsg(start, message);
    if (trajectory.empty())
      throw std::runtime_error("Empty planned segment");
    moveit::core::RobotState result(trajectory.getLastWayPoint());
    result.update();
    return result;
  }

  moveit_msgs::msg::RobotTrajectory handTrajectory(const moveit::core::RobotState& start, double goal)
  {
    auto scene = readScene();
    moveit::core::RobotState state(scene->getCurrentState());
    state.setVariablePositions(start.getVariablePositions());
    robot_trajectory::RobotTrajectory trajectory(model_, hand_group_);
    const double initial = state.getVariablePosition(hand_joint_);
    const int count = std::max(1, static_cast<int>(std::ceil(std::abs(goal - initial) / 0.01)));
    for (int i = 0; i <= count; ++i)
    {
      state.setVariablePosition(hand_joint_, initial + (goal - initial) * i / count);
      state.update();
      if (!state.satisfiesBounds(hand_joint_group_) || scene->isStateColliding(state, "", false))
        throw std::runtime_error("Gripper sweep violates a joint limit or collides, including the table");
      trajectory.addSuffixWayPoint(state, 0.0);
    }
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, hand_velocity_, hand_acceleration_))
      throw std::runtime_error("Gripper time parameterization failed");
    if (trajectory.getDuration() < 0.1)
      trajectory.setWayPointDurationFromPrevious(trajectory.getWayPointCount() - 1, 0.1);
    moveit_msgs::msg::RobotTrajectory message;
    trajectory.getRobotTrajectoryMsg(message);
    return message;
  }

  moveit_msgs::msg::RobotTrajectory planMotion(MoveGroup& move, const moveit::core::RobotState& start,
                                             const Motion& motion)
  {
    if (motion.gripper)
      return handTrajectory(start, motion.joint);
    moveit_msgs::msg::RobotTrajectory message;
    if (!cartesian(move, start, {motion.tcp}, message, motion.label))
      throw std::runtime_error(motion.label + ": full Cartesian path is unavailable");
    retimeAndValidate(start, message, &motion.tcp);
    return message;
  }

  bool pickCycle(MoveGroup& move, const moveit::core::RobotState& measured,
                 const moveit::core::RobotState& home)
  {
    hand_joint_group_ = model_->getJointModelGroup(hand_group_);
    if (!hand_joint_group_ || hand_joint_group_->getActiveJointModelNames() != std::vector<std::string>{hand_joint_})
      throw std::runtime_error("Gripper group must contain the configured single independent joint");
    const auto& bounds = model_->getVariableBounds(hand_joint_);
    if (hand_open_ < bounds.min_position_ || hand_closed_ > bounds.max_position_)
      throw std::runtime_error("Configured gripper positions exceed model limits");
    const double grasp_z = table_z_ + grasp_height_;
    if (grasp_z >= center_.translation().z())
      throw std::runtime_error("Grasp height must be below the home travel height");
    const bool prepare_home = !onPlane(measured);
    if (prepare_home && (!param<bool>(node_, "prepare_home_if_needed", true) || !onPlane(home)))
      throw std::runtime_error("Move to the configured home travel plane before the pick cycle");
    MoveGroup hand(node_, hand_group_, nullptr, rclcpp::Duration::from_seconds(20.0));
    const auto order = visitOrder();
    std::vector<Motion> motions{{"Initial gripper open", true, Transform::Identity(), hand_open_}};
    for (int id : order)
    {
      const Transform high = cells_[static_cast<std::size_t>(id - 1)].tcp;
      Transform low = high;
      low.translation().z() = grasp_z;
      const std::string label = "Cell " + std::to_string(id);
      motions.push_back({label + " APPROACH", false, high, 0.0});
      motions.push_back({label + " DESCEND", false, low, 0.0});
      motions.push_back({label + " CLOSE", true, Transform::Identity(), hand_closed_});
      motions.push_back({label + " LIFT", false, high, 0.0});
      motions.push_back({label + " OPEN", true, Transform::Identity(), hand_open_});
    }
    if (operation_ == "pick_cycle" && param<bool>(node_, "return_to_center", true))
      motions.push_back({"Return to A3 centre", false, center_, 0.0});
    RCLCPP_INFO(logger_, "Pick preflight: %zu cells, block %.1fx%.1fx%.1f mm, table Z=%.6f, grasp TCP Z=%.6f, travel Z=%.6f [%s]",
                order.size(), 1000.0 * block_size_.x(), 1000.0 * block_size_.y(), 1000.0 * block_size_.z(),
                table_z_, grasp_z, center_.translation().z(), frame_.c_str());
    RCLCPP_INFO(logger_, "Table collision checks restored locally; gripper open=%.4f rad, grasp=%.4f rad",
                hand_open_, hand_closed_);
    moveit::core::RobotState predicted(prepare_home ? home : measured);
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = model_->getName();
    moveit::core::robotStateToRobotStateMsg(predicted, display.trajectory_start);
    for (const auto& motion : motions)
    {
      auto planned = planMotion(move, predicted, motion);
      predicted = finalState(predicted, planned, motion.gripper ? hand_group_ : group_);
      display.trajectory.push_back(std::move(planned));
    }
    display_->publish(display);
    RCLCPP_INFO(logger_, "PICK PREFLIGHT PASSED: all %zu segments, including every descend/close/lift/open", motions.size());
    if (!execute_)
    {
      RCLCPP_INFO(logger_, "PLAN ONLY: pick cycle checked; execute:=true enables execution; home preparation needed=%s",
                  prepare_home ? "yes" : "no");
      return true;
    }
    if (prepare_home && !goHome(move, measured, home))
      return false;
    for (const auto& motion : motions)
    {
      if (!rclcpp::ok())
        return false;
      auto current = move.getCurrentState(5.0);
      if (!current)
        throw std::runtime_error("Joint feedback missing before " + motion.label);
      auto trajectory = planMotion(move, *current, motion);
      MoveGroup::Plan plan;
      moveit::core::robotStateToRobotStateMsg(*current, plan.start_state_);
      plan.trajectory_ = trajectory;
      RCLCPP_INFO(logger_, "EXECUTING: %s", motion.label.c_str());
      const auto result = (motion.gripper ? hand : move).execute(plan);
      if (result != moveit::core::MoveItErrorCode::SUCCESS)
        throw std::runtime_error(motion.label + ": execute failed, MoveIt code=" + std::to_string(result.val));
      current = move.getCurrentState(5.0);
      if (!current)
        throw std::runtime_error("Joint feedback missing after " + motion.label);
      current->update();
      if (motion.gripper)
      {
        if (std::abs(current->getVariablePosition(hand_joint_) - motion.joint) > 0.01)
          throw std::runtime_error(motion.label + ": gripper did not reach its target");
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(hand_pause_)));
      }
      else
      {
        const Transform actual = current->getGlobalLinkTransform(frame_).inverse() * current->getGlobalLinkTransform(tcp_);
        if ((actual.translation() - motion.tcp.translation()).norm() > 0.003 ||
            Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(motion.tcp.rotation())) > angle_tolerance_)
          throw std::runtime_error(motion.label + ": TCP did not reach its target");
      }
      RCLCPP_INFO(logger_, "REACHED: %s", motion.label.c_str());
    }
    RCLCPP_INFO(logger_, "SUCCESS: %s complete; %zu cells descended, closed, lifted, and reopened", operation_.c_str(), order.size());
    return true;
  }

  Transform target(double x, double y) const
  {
    Transform result = center_;
    result.translation() += grid_rotation_ * Eigen::Vector3d(x, y, 0.0);
    return result;
  }

  void createCells()
  {
    for (int row = 0; row < rows_; ++row)
      for (int column = 0; column < columns_; ++column)
        cells_.push_back({row * columns_ + column + 1, row + 1, column + 1,
          target(((column + 0.5) / columns_ - 0.5) * size_x_, (0.5 - (row + 0.5) / rows_) * size_y_)});
  }

  bool onPlane(const moveit::core::RobotState& state) const
  {
    moveit::core::RobotState updated(state);
    updated.update();
    const Transform actual = updated.getGlobalLinkTransform(frame_).inverse() * updated.getGlobalLinkTransform(tcp_);
    const double angle = Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(center_.rotation()));
    return std::abs(actual.translation().z() - center_.translation().z()) <= height_tolerance_ && angle <= angle_tolerance_;
  }

  bool cartesian(MoveGroup& move, const moveit::core::RobotState& start,
                 const std::vector<Transform>& tcp_goals, moveit_msgs::msg::RobotTrajectory& trajectory,
                 const std::string& label)
  {
    (void)move;
    // Get the live world/ACM/attached objects, then preserve exact Cartesian samples locally.
    // Humble's Cartesian service applies TOTG resampling, which can round grid corners.
    auto scene = readScene();
    moveit::core::RobotState interpolated(scene->getCurrentState());
    interpolated.setVariablePositions(start.getVariablePositions());
    interpolated.update();
    const Transform world_from_frame = interpolated.getGlobalLinkTransform(frame_);
    EigenSTL::vector_Isometry3d tip_goals;
    for (const auto& goal : tcp_goals)
      tip_goals.push_back(world_from_frame * goal * tip_from_tcp_.inverse());
    const moveit::core::GroupStateValidityCallbackFn valid =
        [scene, this](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
                      const double* values) {
          state->setJointGroupPositions(group, values);
          state->update();
          return state->satisfiesBounds(group) && !scene->isStateColliding(*state, "", false);
        };
    if (scene->isStateColliding(interpolated, "", false))
      throw std::runtime_error(label + ": starting state is in collision");
    std::vector<moveit::core::RobotStatePtr> samples;
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
        &interpolated, joint_group_, samples, model_->getLinkModel(tip_), tip_goals, true,
        moveit::core::MaxEEFStep(step_), moveit::core::JumpThreshold(jump_), valid);
    robot_trajectory::RobotTrajectory raw(model_, group_);
    for (const auto& sample : samples)
      raw.addSuffixWayPoint(sample, 0.0);
    raw.getRobotTrajectoryMsg(trajectory);
    RCLCPP_INFO(logger_, "%s: Cartesian completion %.3f%%, collision checking enabled, samples=%zu",
                label.c_str(), 100.0 * fraction, samples.size());
    return fraction >= 1.0 - 1e-6 && !samples.empty();
  }

  void retimeAndValidate(const moveit::core::RobotState& start, moveit_msgs::msg::RobotTrajectory& message,
                         const Transform* endpoint = nullptr)
  {
    robot_trajectory::RobotTrajectory trajectory(model_, group_);
    trajectory.setRobotTrajectoryMsg(start, message);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, velocity_, acceleration_))
      throw std::runtime_error("Trajectory time parameterization failed");
    moveit::core::RobotState updated_start(start);
    updated_start.update();
    const Transform initial = updated_start.getGlobalLinkTransform(frame_).inverse() * updated_start.getGlobalLinkTransform(tcp_);
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    if (endpoint)
      delta = endpoint->translation() - initial.translation();
    double max_path_error = 0.0, max_angle_error = 0.0, max_step = 0.0;
    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
      trajectory.getWayPointPtr(i)->update();
      const auto& state = trajectory.getWayPoint(i);
      if (!state.satisfiesBounds(joint_group_, 1e-8))
        throw std::runtime_error("Planned state exceeds arm joint limits");
      const Transform actual = state.getGlobalLinkTransform(frame_).inverse() * state.getGlobalLinkTransform(tcp_);
      double path_error = std::abs(actual.translation().z() - center_.translation().z());
      if (endpoint)
      {
        const double fraction = delta.squaredNorm() < 1e-12 ? 0.0 :
            std::clamp((actual.translation() - initial.translation()).dot(delta) / delta.squaredNorm(), 0.0, 1.0);
        path_error = (actual.translation() - (initial.translation() + fraction * delta)).norm();
      }
      max_path_error = std::max(max_path_error, path_error);
      max_angle_error = std::max(max_angle_error,
          Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(center_.rotation())));
      if (i)
        for (const auto& name : joint_group_->getVariableNames())
          max_step = std::max(max_step, std::abs(state.getVariablePosition(name) -
                                                trajectory.getWayPoint(i - 1).getVariablePosition(name)));
    }
    if (max_path_error > height_tolerance_ || max_angle_error > angle_tolerance_ || max_step > max_joint_step_)
      throw std::runtime_error("Planned trajectory violates TCP path/orientation or joint continuity tolerance");
    trajectory.getRobotTrajectoryMsg(message);
    RCLCPP_INFO(logger_, "Validated %.3f s route: max TCP %s error=%.4f mm, angle error=%.6f rad, joint step=%.6f rad",
                trajectory.getDuration(), endpoint ? "line" : "Z", 1000.0 * max_path_error, max_angle_error, max_step);
  }

  bool goHome(MoveGroup& move, const moveit::core::RobotState& current,
              const moveit::core::RobotState& working_home)
  {
    move.clearPoseTargets();
    move.setStartState(current);
    if (!move.setJointValueTarget(working_home))
      throw std::runtime_error("Working-home joint target is invalid");
    MoveGroup::Plan plan;
    const auto result = move.plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Planning to home failed: code=" + std::to_string(result.val));
    if (isPick())
    {
      auto scene = readScene();
      robot_trajectory::RobotTrajectory checked(model_, group_);
      checked.setRobotTrajectoryMsg(current, plan.trajectory_);
      for (std::size_t i = 0; i < checked.getWayPointCount(); ++i)
      {
        checked.getWayPointPtr(i)->update();
        if (scene->isStateColliding(checked.getWayPoint(i), "", false))
          throw std::runtime_error("Home preparation collides with the scene/table; nothing executed");
      }
    }
    showTrajectory(current, plan.trajectory_);
    if (!execute_)
    {
      RCLCPP_INFO(logger_, "PLAN ONLY: working home (orientation=%s)", orientation_mode_.c_str());
      return true;
    }
    const auto executed = move.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Home preparation failed: code=" + std::to_string(executed.val));
    RCLCPP_INFO(logger_, "Working home reached (orientation=%s)", orientation_mode_.c_str());
    return true;
  }

  void showTrajectory(const moveit::core::RobotState& start, const moveit_msgs::msg::RobotTrajectory& trajectory)
  {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = model_->getName();
    moveit::core::robotStateToRobotStateMsg(start, display.trajectory_start);
    display.trajectory.push_back(trajectory);
    display_->publish(display);
  }

  void exportCoordinates()
  {
    std::string file = param<std::string>(node_, "coordinates_file", "");
    if (file.empty())
    {
      const char* home = std::getenv("HOME");
      if (!home)
        throw std::runtime_error("HOME is unset; specify coordinates_file");
      file = (std::filesystem::path(home) / ".ros/ur5e_a3_grid/a3_grid_points.csv").string();
    }
    const std::filesystem::path path(file);
    if (!path.parent_path().empty())
      std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream)
      throw std::runtime_error("Cannot write coordinates_file: " + file);
    stream << "id,row,column,grid_x_m,grid_y_m,frame,x_m,y_m,z_m,qx,qy,qz,qw,grasp_z_m,block_center_z_m\n"
           << std::fixed << std::setprecision(9);
    for (const auto& cell : cells_)
    {
      const auto p = pose(cell.tcp);
      const Eigen::Vector3d local = grid_rotation_.transpose() * (cell.tcp.translation() - center_.translation());
      stream << cell.id << ',' << cell.row << ',' << cell.column << ',' << local.x() << ',' << local.y()
             << ',' << frame_ << ','
             << p.position.x << ',' << p.position.y << ',' << p.position.z << ','
             << p.orientation.x << ',' << p.orientation.y << ',' << p.orientation.z << ',' << p.orientation.w << ','
             << table_z_ + grasp_height_ << ',' << table_z_ + block_size_.z() / 2.0 << '\n';
      RCLCPP_INFO(logger_, "Cell %02d (row=%d,col=%d): TCP XYZ=(%.6f, %.6f, %.6f) [%s]",
                  cell.id, cell.row, cell.column, p.position.x, p.position.y, p.position.z, frame_.c_str());
    }
    stream.close();
    if (!stream)
      throw std::runtime_error("Failed writing coordinates_file: " + file);
    RCLCPP_INFO(logger_, "Coordinates CSV: %s", file.c_str());
  }

  void publishGrid()
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker grid;
    grid.header.frame_id = frame_;
    grid.header.stamp = node_->now();
    grid.ns = "a3_grid"; grid.id = 0;
    grid.type = visualization_msgs::msg::Marker::LINE_LIST;
    grid.action = visualization_msgs::msg::Marker::ADD;
    grid.pose.orientation.w = 1.0;
    grid.scale.x = 0.001;
    grid.color.r = 0.0f; grid.color.g = 0.8f; grid.color.b = 1.0f; grid.color.a = 1.0f;
    for (int c = 0; c <= columns_; ++c)
    {
      const double x = (static_cast<double>(c) / columns_ - 0.5) * size_x_;
      grid.points.push_back(point(target(x, -size_y_ / 2).translation()));
      grid.points.push_back(point(target(x, size_y_ / 2).translation()));
    }
    for (int r = 0; r <= rows_; ++r)
    {
      const double y = (static_cast<double>(r) / rows_ - 0.5) * size_y_;
      grid.points.push_back(point(target(-size_x_ / 2, y).translation()));
      grid.points.push_back(point(target(size_x_ / 2, y).translation()));
    }
    array.markers.push_back(grid);
    for (const auto& cell : cells_)
    {
      auto label = grid;
      label.ns = "a3_cell"; label.id = cell.id; label.points.clear();
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.pose = pose(cell.tcp); label.pose.position.z += 0.008;
      label.scale.x = 0.0; label.scale.y = 0.0; label.scale.z = 0.018;
      label.color.r = 1.0f; label.color.g = 1.0f; label.color.b = 1.0f;
      label.text = std::to_string(cell.id);
      array.markers.push_back(label);
      if (isPick())
      {
        // Reference geometry only: GenericSystem does not simulate grasp/contact/gravity.
        auto block = grid;
        block.ns = "a3_block_reference"; block.id = cell.id; block.points.clear();
        block.type = visualization_msgs::msg::Marker::CUBE;
        block.pose.position = point(cell.tcp.translation());
        block.pose.position.z = table_z_ + block_size_.z() / 2.0;
        const Eigen::Quaterniond rotation(grid_rotation_);
        block.pose.orientation.x = rotation.x(); block.pose.orientation.y = rotation.y();
        block.pose.orientation.z = rotation.z(); block.pose.orientation.w = rotation.w();
        block.scale.x = block_size_.x(); block.scale.y = block_size_.y(); block.scale.z = block_size_.z();
        block.color.r = 0.25f; block.color.g = 0.65f; block.color.b = 0.3f; block.color.a = 0.7f;
        array.markers.push_back(block);
      }
    }
    auto center = grid;
    center.ns = "a3_center"; center.id = 0; center.points.clear();
    center.type = visualization_msgs::msg::Marker::SPHERE;
    center.pose = pose(center_);
    center.scale.x = 0.012; center.scale.y = 0.012; center.scale.z = 0.012;
    center.color.r = 1.0f; center.color.g = 0.5f; center.color.b = 0.0f;
    array.markers.push_back(center);
    markers_->publish(array);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string group_, home_name_, frame_, tip_, tcp_, operation_, hand_group_, hand_joint_, table_link_;
  std::string approach_link_, orientation_mode_;
  bool execute_;
  int columns_, rows_;
  double size_x_, size_y_, step_, jump_, max_joint_step_, height_tolerance_, angle_tolerance_, velocity_, acceleration_;
  double table_z_, grasp_height_, hand_open_, hand_closed_, hand_velocity_, hand_acceleration_, hand_pause_;
  Eigen::Vector3d block_size_;
  Transform center_ = Transform::Identity(), tip_from_tcp_ = Transform::Identity();
  Eigen::Matrix3d grid_rotation_ = Eigen::Matrix3d::Identity();
  std::vector<Cell> cells_;
  moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup* joint_group_ = nullptr;
  const moveit::core::JointModelGroup* hand_joint_group_ = nullptr;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "ur5e_a3_grid", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int result = 1;
  std::unique_ptr<Grid> grid;
  try
  {
    // Own the timer/publishers until after the executor stops.
    grid = std::make_unique<Grid>(node);
    result = grid->run() ? 0 : 1;
    if (result == 0 && param<bool>(node, "keep_alive", true))
    {
      RCLCPP_INFO(node->get_logger(), "Grid remains visible. Ctrl+C closes this client.");
      while (rclcpp::ok())
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    executor.cancel();
    spinner.join();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "A3 grid stopped: %s", e.what());
    executor.cancel();
    spinner.join();
  }
  rclcpp::shutdown();
  return result;
}
