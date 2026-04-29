#pragma once

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <string>
#include <vector>

namespace moveit_trace_motion_planner
{

class TrajectoryBuilder
{
public:
  TrajectoryBuilder(moveit::core::RobotModelConstPtr robot_model, std::string group_name, double waypoint_duration);

  robot_trajectory::RobotTrajectoryPtr build(const std::vector<moveit::core::RobotState>& states) const;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::string group_name_;
  double waypoint_duration_{ 0.1 };
};

}  // namespace moveit_trace_motion_planner
