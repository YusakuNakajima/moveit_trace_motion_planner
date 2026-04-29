#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_trace_motion_planner/trace_planner_types.hpp>

#include <memory>
#include <string>
#include <vector>

namespace moveit_trace_motion_planner
{

class TracePlanner
{
public:
  TracePlanner(moveit::core::RobotModelConstPtr robot_model, TracePlannerConfig config, rclcpp::Logger logger);

  TracePlanResult plan(const planning_scene::PlanningSceneConstPtr& planning_scene,
                       const planning_interface::MotionPlanRequest& request) const;

private:
  bool extractGoalPose(const planning_interface::MotionPlanRequest& request, GoalPose& goal_pose,
                       std::string& error) const;
  bool extractJointGoal(const planning_interface::MotionPlanRequest& request,
                        const moveit::core::JointModelGroup* joint_model_group,
                        const moveit::core::RobotState& start_state, moveit::core::RobotState& goal_state,
                        std::string& error) const;
  std::string getDefaultTipLink(const moveit::core::JointModelGroup* joint_model_group) const;
  bool solveIk(const moveit::core::JointModelGroup* joint_model_group, const std::string& tip_link,
               const Eigen::Isometry3d& target_pose, const moveit::core::RobotState& seed_state,
               moveit::core::RobotState& solution_state) const;
  std::vector<moveit::core::RobotState> buildTraceStatePath(
      const moveit::core::RobotState& start_state, const moveit::core::RobotState& goal_state,
      const moveit::core::JointModelGroup* joint_model_group, const std::string& tip_link,
      const std::vector<TraceWaypoint>& start_waypoints, const std::vector<TraceWaypoint>& goal_waypoints,
      const planning_scene::PlanningSceneConstPtr& planning_scene, TraceFailureReason& failure_reason) const;

  moveit::core::RobotModelConstPtr robot_model_;
  TracePlannerConfig config_;
  rclcpp::Logger logger_;
};

}  // namespace moveit_trace_motion_planner
