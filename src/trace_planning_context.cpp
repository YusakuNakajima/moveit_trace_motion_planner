#include <moveit_trace_motion_planner/trace_planning_context.hpp>

#include <moveit_trace_motion_planner/trajectory_builder.hpp>

#include <utility>

namespace moveit_trace_motion_planner
{

TracePlanningContext::TracePlanningContext(const std::string& name, const std::string& group,
                                           moveit::core::RobotModelConstPtr robot_model, TracePlannerConfig config,
                                           rclcpp::Logger logger)
  : planning_interface::PlanningContext(name, group)
  , robot_model_(std::move(robot_model))
  , config_(std::move(config))
  , logger_(logger)
{
}

bool TracePlanningContext::solve(planning_interface::MotionPlanResponse& response)
{
  TracePlanner planner(robot_model_, config_, logger_);
  TracePlanResult result = planner.plan(planning_scene_, request_);

  response.error_code_ = result.error_code;
  response.planning_time_ = result.planning_time;

  if (!result.success)
  {
    RCLCPP_WARN(logger_, "Trace planner failed: %s", result.message.c_str());
    return false;
  }

  if (result.waypoint_states.empty())
  {
    response.error_code_ = makeErrorCode(moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
    return false;
  }

  TrajectoryBuilder trajectory_builder(robot_model_, request_.group_name, config_.waypoint_duration);
  response.trajectory_ = trajectory_builder.build(result.waypoint_states);
  return true;
}

bool TracePlanningContext::solve(planning_interface::MotionPlanDetailedResponse& response)
{
  planning_interface::MotionPlanResponse simple_response;
  const bool solved = solve(simple_response);

  response.error_code_ = simple_response.error_code_;
  response.processing_time_.push_back(simple_response.planning_time_);
  response.description_.push_back(config_.planner_id);
  if (simple_response.trajectory_)
  {
    response.trajectory_.push_back(simple_response.trajectory_);
  }

  return solved;
}

void TracePlanningContext::clear()
{
}

bool TracePlanningContext::terminate()
{
  return true;
}

}  // namespace moveit_trace_motion_planner
