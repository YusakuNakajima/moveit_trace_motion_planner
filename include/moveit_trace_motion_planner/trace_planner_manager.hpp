#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit_trace_motion_planner/trace_planner_types.hpp>

#include <memory>
#include <string>
#include <vector>

namespace moveit_trace_motion_planner
{

class TracePlannerManager : public planning_interface::PlannerManager
{
public:
  TracePlannerManager() = default;
  ~TracePlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override;

  std::string getDescription() const override;
  void getPlanningAlgorithms(std::vector<std::string>& algorithms) const override;
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& planner_configurations) override;
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& request,
      moveit_msgs::msg::MoveItErrorCodes& error_code) const override;
  bool canServiceRequest(const planning_interface::MotionPlanRequest& request) const override;

private:
  template <typename T>
  void readParameter(const std::string& name, T& value) const;

  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  TracePlannerConfig config_;
};

}  // namespace moveit_trace_motion_planner
