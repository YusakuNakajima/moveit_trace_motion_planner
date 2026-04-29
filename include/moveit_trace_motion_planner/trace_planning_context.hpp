#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_trace_motion_planner/trace_planner.hpp>
#include <moveit_trace_motion_planner/trace_planner_types.hpp>

#include <memory>
#include <string>

namespace moveit_trace_motion_planner
{

class TracePlanningContext : public planning_interface::PlanningContext
{
public:
  TracePlanningContext(const std::string& name, const std::string& group,
                       moveit::core::RobotModelConstPtr robot_model, TracePlannerConfig config,
                       rclcpp::Logger logger);

  bool solve(planning_interface::MotionPlanResponse& response) override;
  bool solve(planning_interface::MotionPlanDetailedResponse& response) override;
  void clear() override;
  bool terminate() override;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  TracePlannerConfig config_;
  rclcpp::Logger logger_;
};

}  // namespace moveit_trace_motion_planner
