#include <moveit_trace_motion_planner/trace_planner_types.hpp>

namespace moveit_trace_motion_planner
{

moveit_msgs::msg::MoveItErrorCodes makeErrorCode(int value)
{
  moveit_msgs::msg::MoveItErrorCodes code;
  code.val = value;
  return code;
}

int toMoveItErrorCode(TraceFailureReason reason)
{
  using moveit_msgs::msg::MoveItErrorCodes;

  switch (reason)
  {
    case TraceFailureReason::NONE:
      return MoveItErrorCodes::SUCCESS;
    case TraceFailureReason::INVALID_REQUEST:
      return MoveItErrorCodes::INVALID_MOTION_PLAN;
    case TraceFailureReason::UNSUPPORTED_REQUEST:
      return MoveItErrorCodes::PLANNING_FAILED;
    case TraceFailureReason::SHIFT_NOT_IMPLEMENTED:
      return MoveItErrorCodes::PLANNING_FAILED;
    case TraceFailureReason::GOAL_IK_FAILED:
      return MoveItErrorCodes::NO_IK_SOLUTION;
    case TraceFailureReason::INTERMEDIATE_IK_FAILED:
      return MoveItErrorCodes::NO_IK_SOLUTION;
    case TraceFailureReason::COLLISION_FREE_PATH_NOT_FOUND:
      return MoveItErrorCodes::PLANNING_FAILED;
    case TraceFailureReason::TIMEOUT:
      return MoveItErrorCodes::TIMED_OUT;
  }

  return MoveItErrorCodes::PLANNING_FAILED;
}

std::string toString(TraceFailureReason reason)
{
  switch (reason)
  {
    case TraceFailureReason::NONE:
      return "none";
    case TraceFailureReason::INVALID_REQUEST:
      return "invalid request";
    case TraceFailureReason::UNSUPPORTED_REQUEST:
      return "unsupported request";
    case TraceFailureReason::SHIFT_NOT_IMPLEMENTED:
      return "shift motion is not implemented in v0";
    case TraceFailureReason::GOAL_IK_FAILED:
      return "goal IK failed";
    case TraceFailureReason::INTERMEDIATE_IK_FAILED:
      return "intermediate IK failed";
    case TraceFailureReason::COLLISION_FREE_PATH_NOT_FOUND:
      return "collision-free path not found";
    case TraceFailureReason::TIMEOUT:
      return "planning timed out";
  }

  return "unknown failure";
}

}  // namespace moveit_trace_motion_planner
