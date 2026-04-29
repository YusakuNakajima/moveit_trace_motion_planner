#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <string>
#include <vector>

namespace moveit_trace_motion_planner
{

enum class TraceFailureReason
{
  NONE,
  INVALID_REQUEST,
  UNSUPPORTED_REQUEST,
  SHIFT_NOT_IMPLEMENTED,
  GOAL_IK_FAILED,
  INTERMEDIATE_IK_FAILED,
  COLLISION_FREE_PATH_NOT_FOUND,
  TIMEOUT
};

struct TracePlannerConfig
{
  std::string planner_id{ "TraceMotion" };
  double max_planning_time{ 1.0 };
  std::size_t max_waypoints{ 20 };

  double ik_timeout{ 0.02 };
  int ik_attempts{ 3 };

  double max_joint_step{ 0.03 };
  double waypoint_duration{ 0.1 };

  bool check_self_collision{ true };
  bool check_scene_collision{ true };

  bool use_joint_positions{ true };
  bool use_link_directions{ true };
  bool skip_base_link{ true };
  bool skip_tip_link{ false };

  bool shift_motion_enabled{ false };
};

struct TraceWaypoint
{
  enum class Source
  {
    START,
    GOAL
  };

  Source source{ Source::START };
  std::string link_name;
  Eigen::Isometry3d target_pose{ Eigen::Isometry3d::Identity() };
};

struct GoalPose
{
  std::string link_name;
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };
};

struct TracePlanResult
{
  bool success{ false };
  TraceFailureReason failure_reason{ TraceFailureReason::NONE };
  moveit_msgs::msg::MoveItErrorCodes error_code;
  std::string message;
  std::vector<moveit::core::RobotState> waypoint_states;
  double planning_time{ 0.0 };
};

moveit_msgs::msg::MoveItErrorCodes makeErrorCode(int value);
int toMoveItErrorCode(TraceFailureReason reason);
std::string toString(TraceFailureReason reason);

}  // namespace moveit_trace_motion_planner
