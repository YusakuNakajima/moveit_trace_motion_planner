#include <moveit_trace_motion_planner/trace_planner_manager.hpp>

#include <moveit_trace_motion_planner/trace_planning_context.hpp>

#include <class_loader/class_loader.hpp>

#include <algorithm>
#include <utility>

namespace moveit_trace_motion_planner
{

bool TracePlannerManager::initialize(const moveit::core::RobotModelConstPtr& model,
                                     const rclcpp::Node::SharedPtr& node,
                                     const std::string& parameter_namespace)
{
  robot_model_ = model;
  node_ = node;
  parameter_namespace_ = parameter_namespace;

  readParameter("planner_id", config_.planner_id);
  readParameter("trace_planner.max_planning_time", config_.max_planning_time);
  readParameter("trace_planner.ik.timeout", config_.ik_timeout);
  readParameter("trace_planner.ik.attempts", config_.ik_attempts);
  readParameter("trace_planner.ik.try_alternate_seeds", config_.try_alternate_ik_seeds);
  readParameter("trace_planner.direction_ik.enabled", config_.direction_ik.enabled);
  readParameter("trace_planner.direction_ik.max_iterations", config_.direction_ik.max_iterations);
  readParameter("trace_planner.direction_ik.timeout", config_.direction_ik.timeout);
  readParameter("trace_planner.direction_ik.damping", config_.direction_ik.damping);
  readParameter("trace_planner.direction_ik.step_size", config_.direction_ik.step_size);
  readParameter("trace_planner.direction_ik.position_tolerance", config_.direction_ik.position_tolerance);
  readParameter("trace_planner.direction_ik.direction_tolerance", config_.direction_ik.direction_tolerance);
  readParameter("trace_planner.direction_ik.max_joint_update", config_.direction_ik.max_joint_update);
  readParameter("trace_planner.interpolation.max_joint_step", config_.max_joint_step);
  readParameter("trace_planner.trajectory.waypoint_duration", config_.waypoint_duration);
  readParameter("trace_planner.waypoint_source.use_joint_positions", config_.use_joint_positions);
  readParameter("trace_planner.waypoint_source.use_link_directions", config_.use_link_directions);
  readParameter("trace_planner.waypoint_source.skip_base_link", config_.skip_base_link);
  readParameter("trace_planner.waypoint_source.skip_tip_link", config_.skip_tip_link);
  readParameter("trace_planner.collision.check_self_collision", config_.check_self_collision);
  readParameter("trace_planner.collision.check_scene_collision", config_.check_scene_collision);
  readParameter("trace_planner.shift_motion.enabled", config_.shift_motion_enabled);

  int max_waypoints = static_cast<int>(config_.max_waypoints);
  readParameter("trace_planner.max_waypoints", max_waypoints);
  config_.max_waypoints = static_cast<std::size_t>(std::max(0, max_waypoints));

  if (!robot_model_ || !node_)
  {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Initialized Trace motion planner plugin with planner id '%s'",
              config_.planner_id.c_str());
  return true;
}

std::string TracePlannerManager::getDescription() const
{
  return "Trace Motion Planner";
}

void TracePlannerManager::getPlanningAlgorithms(std::vector<std::string>& algorithms) const
{
  algorithms.clear();
  algorithms.push_back(config_.planner_id);
}

void TracePlannerManager::setPlannerConfigurations(
    const planning_interface::PlannerConfigurationMap& planner_configurations)
{
  config_settings_ = planner_configurations;
}

planning_interface::PlanningContextPtr TracePlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene, const planning_interface::MotionPlanRequest& request,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (!canServiceRequest(request))
  {
    error_code = makeErrorCode(moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
    return planning_interface::PlanningContextPtr();
  }

  auto context = std::make_shared<TracePlanningContext>(config_.planner_id, request.group_name, robot_model_, config_,
                                                       node_->get_logger());
  context->setPlanningScene(planning_scene);
  context->setMotionPlanRequest(request);
  error_code = makeErrorCode(moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
  return context;
}

bool TracePlannerManager::canServiceRequest(const planning_interface::MotionPlanRequest& request) const
{
  if (!robot_model_)
  {
    return false;
  }

  if (!request.planner_id.empty() && request.planner_id != config_.planner_id)
  {
    return false;
  }

  if (request.group_name.empty())
  {
    return false;
  }

  return robot_model_->getJointModelGroup(request.group_name) != nullptr;
}

template <typename T>
void TracePlannerManager::readParameter(const std::string& name, T& value) const
{
  if (!node_)
  {
    return;
  }

  std::vector<std::string> candidates;
  if (!parameter_namespace_.empty())
  {
    candidates.push_back(parameter_namespace_ + "." + name);
  }
  candidates.push_back(name);
  candidates.push_back("trace_motion_planner." + name);

  for (const auto& parameter_name : candidates)
  {
    try
    {
      if (node_->get_parameter(parameter_name, value))
      {
        return;
      }
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException&)
    {
    }
  }
}

}  // namespace moveit_trace_motion_planner

CLASS_LOADER_REGISTER_CLASS(moveit_trace_motion_planner::TracePlannerManager, planning_interface::PlannerManager)
