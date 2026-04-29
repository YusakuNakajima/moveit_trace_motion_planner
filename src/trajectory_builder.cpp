#include <moveit_trace_motion_planner/trajectory_builder.hpp>

#include <memory>
#include <utility>

namespace moveit_trace_motion_planner
{

TrajectoryBuilder::TrajectoryBuilder(moveit::core::RobotModelConstPtr robot_model, std::string group_name,
                                     double waypoint_duration)
  : robot_model_(std::move(robot_model)), group_name_(std::move(group_name)), waypoint_duration_(waypoint_duration)
{
}

robot_trajectory::RobotTrajectoryPtr
TrajectoryBuilder::build(const std::vector<moveit::core::RobotState>& states) const
{
  auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, group_name_);

  bool first = true;
  for (const auto& state : states)
  {
    trajectory->addSuffixWayPoint(state, first ? 0.0 : waypoint_duration_);
    first = false;
  }

  return trajectory;
}

}  // namespace moveit_trace_motion_planner
