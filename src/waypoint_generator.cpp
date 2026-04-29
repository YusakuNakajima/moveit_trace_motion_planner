#include <moveit_trace_motion_planner/waypoint_generator.hpp>

#include <algorithm>
#include <utility>

namespace moveit_trace_motion_planner
{

WaypointGenerator::WaypointGenerator(TracePlannerConfig config) : config_(std::move(config))
{
}

std::vector<TraceWaypoint>
WaypointGenerator::generate(const moveit::core::RobotState& state,
                            const moveit::core::JointModelGroup* joint_model_group, const std::string& tip_link,
                            TraceWaypoint::Source source) const
{
  std::vector<TraceWaypoint> waypoints;
  if (!joint_model_group || !config_.use_joint_positions)
  {
    return waypoints;
  }

  std::vector<std::string> link_names = joint_model_group->getLinkModelNames();
  std::reverse(link_names.begin(), link_names.end());
  const Eigen::Matrix3d fallback_orientation = state.getGlobalLinkTransform(tip_link).linear();

  for (const std::string& link_name : link_names)
  {
    if (config_.skip_tip_link && link_name == tip_link)
    {
      continue;
    }

    const auto* link_model = state.getRobotModel()->getLinkModel(link_name);
    if (!link_model)
    {
      continue;
    }

    if (config_.skip_base_link && link_model->getParentJointModel() &&
        link_model->getParentJointModel()->getType() == moveit::core::JointModel::FIXED)
    {
      continue;
    }

    TraceWaypoint waypoint;
    waypoint.source = source;
    waypoint.link_name = link_name;
    waypoint.target_pose = state.getGlobalLinkTransform(link_name);
    if (!config_.use_link_directions)
    {
      waypoint.target_pose.linear() = fallback_orientation;
    }
    waypoints.push_back(waypoint);
  }

  return waypoints;
}

}  // namespace moveit_trace_motion_planner
