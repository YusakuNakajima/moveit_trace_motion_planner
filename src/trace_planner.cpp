#include <moveit_trace_motion_planner/trace_planner.hpp>

#include <moveit_trace_motion_planner/collision_checker.hpp>
#include <moveit_trace_motion_planner/direction_ik_solver.hpp>
#include <moveit_trace_motion_planner/trajectory_builder.hpp>
#include <moveit_trace_motion_planner/waypoint_generator.hpp>

#include <moveit/robot_state/conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace moveit_trace_motion_planner
{
namespace
{

bool hasPathConstraints(const moveit_msgs::msg::Constraints& constraints)
{
  return !constraints.joint_constraints.empty() || !constraints.position_constraints.empty() ||
         !constraints.orientation_constraints.empty() || !constraints.visibility_constraints.empty();
}

Eigen::Isometry3d toEigenPose(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (quaternion.norm() > 0.0)
  {
    quaternion.normalize();
    transform.linear() = quaternion.toRotationMatrix();
  }

  return transform;
}

Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::msg::Quaternion& quaternion_msg)
{
  Eigen::Quaterniond quaternion(quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z);
  if (quaternion.norm() > 0.0)
  {
    quaternion.normalize();
  }
  return quaternion;
}

}  // namespace

TracePlanner::TracePlanner(moveit::core::RobotModelConstPtr robot_model, TracePlannerConfig config,
                           rclcpp::Logger logger)
  : robot_model_(std::move(robot_model)), config_(std::move(config)), logger_(logger)
{
}

TracePlanResult TracePlanner::plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   const planning_interface::MotionPlanRequest& request) const
{
  const auto started_at = std::chrono::steady_clock::now();

  auto finish = [&](TracePlanResult result) {
    const auto finished_at = std::chrono::steady_clock::now();
    result.planning_time = std::chrono::duration<double>(finished_at - started_at).count();
    result.error_code = makeErrorCode(toMoveItErrorCode(result.failure_reason));
    return result;
  };

  TracePlanResult result;
  result.failure_reason = TraceFailureReason::INVALID_REQUEST;

  if (!planning_scene)
  {
    result.message = "Planning scene is null";
    return finish(result);
  }

  if (config_.shift_motion_enabled)
  {
    result.failure_reason = TraceFailureReason::SHIFT_NOT_IMPLEMENTED;
    result.message = "Shift motion is configured on, but v0 only implements Trace motion";
    return finish(result);
  }

  if (request.group_name.empty())
  {
    result.message = "Planning group is empty";
    return finish(result);
  }

  if (hasPathConstraints(request.path_constraints))
  {
    result.failure_reason = TraceFailureReason::UNSUPPORTED_REQUEST;
    result.message = "Path constraints are not supported by the v0 Trace planner";
    return finish(result);
  }

  const auto* joint_model_group = robot_model_->getJointModelGroup(request.group_name);
  if (!joint_model_group)
  {
    result.message = "Unknown planning group: " + request.group_name;
    return finish(result);
  }

  moveit::core::RobotState start_state(planning_scene->getCurrentState());
  moveit::core::robotStateMsgToRobotState(request.start_state, start_state);
  start_state.update();
  start_state.enforceBounds(joint_model_group);

  moveit::core::RobotState goal_state(start_state);
  std::string tip_link;

  GoalPose goal_pose;
  std::string pose_goal_error;
  if (extractGoalPose(request, goal_pose, pose_goal_error))
  {
    tip_link = goal_pose.link_name;
    if (!solveIk(joint_model_group, tip_link, goal_pose.pose, start_state, goal_state))
    {
      result.failure_reason = TraceFailureReason::GOAL_IK_FAILED;
      result.message = "Failed to solve IK for the goal pose";
      return finish(result);
    }
  }
  else
  {
    std::string joint_goal_error;
    if (!extractJointGoal(request, joint_model_group, start_state, goal_state, joint_goal_error))
    {
      result.message = pose_goal_error + "; " + joint_goal_error;
      return finish(result);
    }
    tip_link = getDefaultTipLink(joint_model_group);
    if (tip_link.empty())
    {
      result.failure_reason = TraceFailureReason::UNSUPPORTED_REQUEST;
      result.message = "Could not infer a tip link for the planning group";
      return finish(result);
    }
  }

  CollisionChecker collision_checker(planning_scene, request.group_name, config_.max_joint_step);
  std::vector<moveit::core::RobotState> direct_path{ start_state, goal_state };
  if (collision_checker.isPathCollisionFree(direct_path))
  {
    result.success = true;
    result.failure_reason = TraceFailureReason::NONE;
    result.message = "Direct joint interpolation is collision-free";
    result.waypoint_states = std::move(direct_path);
    return finish(result);
  }

  WaypointGenerator waypoint_generator(config_);
  const auto start_waypoints =
      waypoint_generator.generate(start_state, joint_model_group, tip_link, TraceWaypoint::Source::START);
  const auto goal_waypoints =
      waypoint_generator.generate(goal_state, joint_model_group, tip_link, TraceWaypoint::Source::GOAL);

  TraceFailureReason trace_failure = TraceFailureReason::COLLISION_FREE_PATH_NOT_FOUND;
  auto trace_path = buildTraceStatePath(start_state, goal_state, joint_model_group, tip_link, start_waypoints,
                                        goal_waypoints, planning_scene, trace_failure);
  if (!trace_path.empty())
  {
    result.success = true;
    result.failure_reason = TraceFailureReason::NONE;
    result.message = "Trace waypoint path is collision-free";
    result.waypoint_states = std::move(trace_path);
    return finish(result);
  }

  result.failure_reason = trace_failure;
  result.message = toString(trace_failure);
  return finish(result);
}

bool TracePlanner::extractGoalPose(const planning_interface::MotionPlanRequest& request, GoalPose& goal_pose,
                                   std::string& error) const
{
  if (request.goal_constraints.size() != 1)
  {
    error = "Trace planner v0 requires exactly one goal constraint";
    return false;
  }

  const auto& constraints = request.goal_constraints.front();
  if (constraints.position_constraints.size() != 1 || constraints.orientation_constraints.size() != 1)
  {
    error = "Trace planner v0 requires one position constraint and one orientation constraint";
    return false;
  }

  const auto& position_constraint = constraints.position_constraints.front();
  const auto& orientation_constraint = constraints.orientation_constraints.front();

  if (position_constraint.link_name.empty() || orientation_constraint.link_name.empty() ||
      position_constraint.link_name != orientation_constraint.link_name)
  {
    error = "Goal position and orientation constraints must refer to the same link";
    return false;
  }

  if (position_constraint.constraint_region.primitive_poses.empty())
  {
    error = "Goal position constraint has no primitive pose";
    return false;
  }

  goal_pose.link_name = position_constraint.link_name;
  goal_pose.pose = toEigenPose(position_constraint.constraint_region.primitive_poses.front());
  goal_pose.pose.linear() = toEigenQuaternion(orientation_constraint.orientation).toRotationMatrix();
  return true;
}

bool TracePlanner::extractJointGoal(const planning_interface::MotionPlanRequest& request,
                                    const moveit::core::JointModelGroup* joint_model_group,
                                    const moveit::core::RobotState& start_state,
                                    moveit::core::RobotState& goal_state, std::string& error) const
{
  if (request.goal_constraints.size() != 1)
  {
    error = "joint goal requires exactly one goal constraint";
    return false;
  }

  const auto& constraints = request.goal_constraints.front();
  if (constraints.joint_constraints.empty())
  {
    error = "joint goal has no joint constraints";
    return false;
  }

  if (!constraints.position_constraints.empty() || !constraints.orientation_constraints.empty() ||
      !constraints.visibility_constraints.empty())
  {
    error = "mixed joint/pose goal constraints are not supported";
    return false;
  }

  const std::vector<std::string>& group_variables = joint_model_group->getVariableNames();
  goal_state = start_state;

  for (const auto& joint_constraint : constraints.joint_constraints)
  {
    if (std::find(group_variables.begin(), group_variables.end(), joint_constraint.joint_name) == group_variables.end())
    {
      error = "joint goal contains a joint outside the planning group: " + joint_constraint.joint_name;
      return false;
    }
    goal_state.setVariablePosition(joint_constraint.joint_name, joint_constraint.position);
  }

  goal_state.enforceBounds(joint_model_group);
  goal_state.update();
  return true;
}

std::string TracePlanner::getDefaultTipLink(const moveit::core::JointModelGroup* joint_model_group) const
{
  const std::vector<std::string>& link_names = joint_model_group->getLinkModelNames();
  if (link_names.empty())
  {
    return {};
  }
  return link_names.back();
}

bool TracePlanner::solveIk(const moveit::core::JointModelGroup* joint_model_group, const std::string& tip_link,
                           const Eigen::Isometry3d& target_pose, const moveit::core::RobotState& seed_state,
                           moveit::core::RobotState& solution_state) const
{
  const int attempts = std::max(1, config_.ik_attempts);
  for (int attempt = 0; attempt < attempts; ++attempt)
  {
    solution_state = seed_state;
    if (solution_state.setFromIK(joint_model_group, target_pose, tip_link, config_.ik_timeout))
    {
      solution_state.enforceBounds(joint_model_group);
      solution_state.update();
      return true;
    }
  }

  return false;
}

bool TracePlanner::solveIkWithSeeds(const moveit::core::JointModelGroup* joint_model_group,
                                    const std::string& tip_link, const Eigen::Isometry3d& target_pose,
                                    const std::vector<const moveit::core::RobotState*>& seed_states,
                                    moveit::core::RobotState& solution_state, bool allow_direction_ik) const
{
  DirectionIkSolver direction_ik_solver(config_.direction_ik);
  for (const auto* seed_state : seed_states)
  {
    if (!seed_state)
    {
      continue;
    }
    if (allow_direction_ik && direction_ik_solver.solve(joint_model_group, tip_link, target_pose, *seed_state,
                                                        solution_state))
    {
      return true;
    }
    if (solveIk(joint_model_group, tip_link, target_pose, *seed_state, solution_state))
    {
      return true;
    }
    if (!config_.try_alternate_ik_seeds)
    {
      break;
    }
  }

  return false;
}

std::vector<moveit::core::RobotState> TracePlanner::buildTraceStatePath(
    const moveit::core::RobotState& start_state, const moveit::core::RobotState& goal_state,
    const moveit::core::JointModelGroup* joint_model_group, const std::string& tip_link,
    const std::vector<TraceWaypoint>& start_waypoints, const std::vector<TraceWaypoint>& goal_waypoints,
    const planning_scene::PlanningSceneConstPtr& planning_scene, TraceFailureReason& failure_reason) const
{
  CollisionChecker collision_checker(planning_scene, joint_model_group->getName(), config_.max_joint_step);

  std::vector<moveit::core::RobotState> start_side;
  std::vector<moveit::core::RobotState> goal_side;

  const std::size_t available_waypoints = start_waypoints.size() + goal_waypoints.size();
  const std::size_t max_waypoints = std::min(config_.max_waypoints, available_waypoints);
  bool saw_intermediate_ik_success = false;
  std::size_t attempted_waypoints = 0;
  std::size_t ik_failures = 0;
  std::size_t collision_failures = 0;

  for (std::size_t added = 0; added < max_waypoints; ++added)
  {
    const bool use_goal_waypoint = (added % 2 == 0);
    const std::size_t index = added / 2;

    const TraceWaypoint* waypoint = nullptr;
    if (use_goal_waypoint && index < goal_waypoints.size())
    {
      waypoint = &goal_waypoints[index];
    }
    else if (!use_goal_waypoint && index < start_waypoints.size())
    {
      waypoint = &start_waypoints[index];
    }
    else if (index < goal_waypoints.size())
    {
      waypoint = &goal_waypoints[index];
    }
    else if (index < start_waypoints.size())
    {
      waypoint = &start_waypoints[index];
    }

    if (!waypoint)
    {
      break;
    }

    ++attempted_waypoints;
    const moveit::core::RobotState& seed_state =
        waypoint->source == TraceWaypoint::Source::GOAL ? goal_state : start_state;
    const moveit::core::RobotState* same_side_seed = &seed_state;
    const moveit::core::RobotState* last_same_side_seed = nullptr;
    const moveit::core::RobotState* opposite_side_seed =
        waypoint->source == TraceWaypoint::Source::GOAL ? &start_state : &goal_state;
    if (waypoint->source == TraceWaypoint::Source::GOAL && !goal_side.empty())
    {
      last_same_side_seed = &goal_side.front();
    }
    else if (waypoint->source == TraceWaypoint::Source::START && !start_side.empty())
    {
      last_same_side_seed = &start_side.back();
    }

    moveit::core::RobotState intermediate_state(seed_state);
    const std::vector<const moveit::core::RobotState*> seed_states = { same_side_seed, last_same_side_seed,
                                                                       opposite_side_seed };
    if (!solveIkWithSeeds(joint_model_group, tip_link, waypoint->target_pose, seed_states, intermediate_state, true))
    {
      ++ik_failures;
      RCLCPP_DEBUG(logger_, "Trace waypoint IK failed for link '%s'", waypoint->link_name.c_str());
      continue;
    }

    saw_intermediate_ik_success = true;
    if (waypoint->source == TraceWaypoint::Source::GOAL)
    {
      goal_side.insert(goal_side.begin(), intermediate_state);
    }
    else
    {
      start_side.push_back(intermediate_state);
    }

    std::vector<moveit::core::RobotState> candidate_path;
    candidate_path.reserve(2 + start_side.size() + goal_side.size());
    candidate_path.push_back(start_state);
    candidate_path.insert(candidate_path.end(), start_side.begin(), start_side.end());
    candidate_path.insert(candidate_path.end(), goal_side.begin(), goal_side.end());
    candidate_path.push_back(goal_state);

    if (collision_checker.isPathCollisionFree(candidate_path))
    {
      failure_reason = TraceFailureReason::NONE;
      RCLCPP_INFO(logger_, "Trace planner accepted %zu waypoint(s) after trying %zu candidate waypoint(s)",
                  candidate_path.size() - 2, attempted_waypoints);
      return candidate_path;
    }

    ++collision_failures;
  }

  failure_reason = saw_intermediate_ik_success ? TraceFailureReason::COLLISION_FREE_PATH_NOT_FOUND :
                                                 TraceFailureReason::INTERMEDIATE_IK_FAILED;
  RCLCPP_WARN(logger_,
              "Trace planner exhausted waypoint candidates: attempted=%zu, ik_failures=%zu, collision_failures=%zu, "
              "start_candidates=%zu, goal_candidates=%zu, max_waypoints=%zu",
              attempted_waypoints, ik_failures, collision_failures, start_waypoints.size(), goal_waypoints.size(),
              max_waypoints);
  return {};
}

}  // namespace moveit_trace_motion_planner
