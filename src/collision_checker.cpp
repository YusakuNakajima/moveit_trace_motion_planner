#include <moveit_trace_motion_planner/collision_checker.hpp>

#include <algorithm>
#include <cmath>
#include <moveit/collision_detection/collision_common.h>

namespace moveit_trace_motion_planner
{

CollisionChecker::CollisionChecker(planning_scene::PlanningSceneConstPtr planning_scene, std::string group_name,
                                   double max_joint_step)
  : planning_scene_(std::move(planning_scene)), group_name_(std::move(group_name)), max_joint_step_(max_joint_step)
{
  if (planning_scene_)
  {
    joint_model_group_ = planning_scene_->getRobotModel()->getJointModelGroup(group_name_);
  }
}

bool CollisionChecker::isStateCollisionFree(const moveit::core::RobotState& state) const
{
  if (!planning_scene_)
  {
    return false;
  }

  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.group_name = group_name_;
  request.contacts = false;
  request.verbose = false;

  planning_scene_->checkCollision(request, result, state);
  return !result.collision;
}

bool CollisionChecker::isSegmentCollisionFree(const moveit::core::RobotState& from,
                                              const moveit::core::RobotState& to) const
{
  if (!joint_model_group_)
  {
    return false;
  }

  const int steps = segmentSteps(from, to);
  moveit::core::RobotState sampled(from);

  for (int i = 0; i <= steps; ++i)
  {
    const double t = steps == 0 ? 1.0 : static_cast<double>(i) / static_cast<double>(steps);
    from.interpolate(to, t, sampled);
    sampled.update();
    if (!isStateCollisionFree(sampled))
    {
      return false;
    }
  }

  return true;
}

bool CollisionChecker::isPathCollisionFree(const std::vector<moveit::core::RobotState>& states) const
{
  if (states.empty())
  {
    return false;
  }

  if (!isStateCollisionFree(states.front()))
  {
    return false;
  }

  for (std::size_t i = 1; i < states.size(); ++i)
  {
    if (!isSegmentCollisionFree(states[i - 1], states[i]))
    {
      return false;
    }
  }

  return true;
}

int CollisionChecker::segmentSteps(const moveit::core::RobotState& from, const moveit::core::RobotState& to) const
{
  if (!joint_model_group_ || max_joint_step_ <= 0.0)
  {
    return 1;
  }

  std::vector<double> from_values;
  std::vector<double> to_values;
  from.copyJointGroupPositions(joint_model_group_, from_values);
  to.copyJointGroupPositions(joint_model_group_, to_values);

  double max_delta = 0.0;
  const std::size_t count = std::min(from_values.size(), to_values.size());
  for (std::size_t i = 0; i < count; ++i)
  {
    max_delta = std::max(max_delta, std::abs(to_values[i] - from_values[i]));
  }

  return std::max(1, static_cast<int>(std::ceil(max_delta / max_joint_step_)));
}

}  // namespace moveit_trace_motion_planner
