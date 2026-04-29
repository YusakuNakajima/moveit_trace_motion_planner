#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>

#include <string>

namespace moveit_trace_motion_planner
{

struct DirectionIkConfig
{
  bool enabled{ true };
  int max_iterations{ 80 };
  double timeout{ 0.02 };
  double damping{ 0.05 };
  double step_size{ 0.6 };
  double position_tolerance{ 0.005 };
  double direction_tolerance{ 0.02 };
  double max_joint_update{ 0.15 };
};

class DirectionIkSolver
{
public:
  explicit DirectionIkSolver(DirectionIkConfig config);

  bool solve(const moveit::core::JointModelGroup* joint_model_group, const std::string& tip_link,
             const Eigen::Isometry3d& target_pose, const moveit::core::RobotState& seed_state,
             moveit::core::RobotState& solution_state) const;

private:
  Eigen::Vector3d directionError(const Eigen::Vector3d& target_direction,
                                 const Eigen::Vector3d& current_direction) const;

  DirectionIkConfig config_;
};

}  // namespace moveit_trace_motion_planner
