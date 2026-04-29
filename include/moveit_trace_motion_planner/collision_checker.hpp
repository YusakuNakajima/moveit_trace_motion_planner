#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include <string>
#include <vector>

namespace moveit_trace_motion_planner
{

class CollisionChecker
{
public:
  CollisionChecker(planning_scene::PlanningSceneConstPtr planning_scene, std::string group_name, double max_joint_step);

  bool isStateCollisionFree(const moveit::core::RobotState& state) const;
  bool isSegmentCollisionFree(const moveit::core::RobotState& from, const moveit::core::RobotState& to) const;
  bool isPathCollisionFree(const std::vector<moveit::core::RobotState>& states) const;

private:
  int segmentSteps(const moveit::core::RobotState& from, const moveit::core::RobotState& to) const;

  planning_scene::PlanningSceneConstPtr planning_scene_;
  std::string group_name_;
  const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
  double max_joint_step_{ 0.03 };
};

}  // namespace moveit_trace_motion_planner
