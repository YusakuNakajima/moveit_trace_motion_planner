#pragma once

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_trace_motion_planner/trace_planner_types.hpp>

#include <string>
#include <vector>

namespace moveit_trace_motion_planner
{

class WaypointGenerator
{
public:
  explicit WaypointGenerator(TracePlannerConfig config);

  std::vector<TraceWaypoint> generate(const moveit::core::RobotState& state,
                                      const moveit::core::JointModelGroup* joint_model_group,
                                      const std::string& tip_link,
                                      TraceWaypoint::Source source) const;

private:
  TracePlannerConfig config_;
};

}  // namespace moveit_trace_motion_planner
