#include <moveit_trace_motion_planner/direction_ik_solver.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace moveit_trace_motion_planner
{
namespace
{

Eigen::MatrixXd dampedLeastSquaresPseudoInverse(const Eigen::MatrixXd& jacobian, double damping)
{
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
  const Eigen::MatrixXd regularized = jacobian * jacobian.transpose() + damping * damping * identity;
  return jacobian.transpose() * regularized.ldlt().solve(identity);
}

}  // namespace

DirectionIkSolver::DirectionIkSolver(DirectionIkConfig config) : config_(config)
{
}

bool DirectionIkSolver::solve(const moveit::core::JointModelGroup* joint_model_group, const std::string& tip_link,
                              const Eigen::Isometry3d& target_pose, const moveit::core::RobotState& seed_state,
                              moveit::core::RobotState& solution_state) const
{
  if (!config_.enabled || !joint_model_group)
  {
    return false;
  }

  const auto* tip_link_model = seed_state.getRobotModel()->getLinkModel(tip_link);
  if (!tip_link_model)
  {
    return false;
  }

  const auto started_at = std::chrono::steady_clock::now();
  const Eigen::Vector3d target_position = target_pose.translation();
  const Eigen::Vector3d target_direction = target_pose.linear().col(2).normalized();
  moveit::core::RobotState state(seed_state);
  state.update();

  for (int iteration = 0; iteration < config_.max_iterations; ++iteration)
  {
    if (config_.timeout > 0.0)
    {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - started_at).count();
      if (elapsed > config_.timeout)
      {
        break;
      }
    }

    const Eigen::Isometry3d& tip_transform = state.getGlobalLinkTransform(tip_link);
    const Eigen::Vector3d current_position = tip_transform.translation();
    const Eigen::Vector3d current_direction = tip_transform.linear().col(2).normalized();

    const Eigen::Vector3d position_error = target_position - current_position;
    const Eigen::Vector3d direction_error = directionError(target_direction, current_direction);
    if (position_error.norm() <= config_.position_tolerance &&
        direction_error.norm() <= config_.direction_tolerance)
    {
      solution_state = state;
      solution_state.enforceBounds(joint_model_group);
      solution_state.update();
      return true;
    }

    Eigen::MatrixXd full_jacobian;
    if (!state.getJacobian(joint_model_group, tip_link_model, Eigen::Vector3d::Zero(), full_jacobian))
    {
      return false;
    }
    if (full_jacobian.rows() < 6)
    {
      return false;
    }

    Eigen::MatrixXd task_jacobian(6, full_jacobian.cols());
    task_jacobian.topRows<3>() = full_jacobian.topRows<3>();
    task_jacobian.bottomRows<3>() = full_jacobian.bottomRows<3>();

    Eigen::VectorXd error(6);
    error.head<3>() = position_error;
    error.tail<3>() = direction_error;

    Eigen::VectorXd delta = config_.step_size * dampedLeastSquaresPseudoInverse(task_jacobian, config_.damping) * error;
    for (Eigen::Index i = 0; i < delta.size(); ++i)
    {
      delta[i] = std::clamp(delta[i], -config_.max_joint_update, config_.max_joint_update);
    }

    std::vector<double> values;
    state.copyJointGroupPositions(joint_model_group, values);
    if (values.size() != static_cast<std::size_t>(delta.size()))
    {
      return false;
    }
    for (std::size_t i = 0; i < values.size(); ++i)
    {
      values[i] += delta[static_cast<Eigen::Index>(i)];
    }

    state.setJointGroupPositions(joint_model_group, values);
    state.enforceBounds(joint_model_group);
    state.update();
  }

  return false;
}

Eigen::Vector3d DirectionIkSolver::directionError(const Eigen::Vector3d& target_direction,
                                                  const Eigen::Vector3d& current_direction) const
{
  const Eigen::Vector3d axis = current_direction.cross(target_direction);
  const double sin_angle = axis.norm();
  const double cos_angle = std::clamp(current_direction.dot(target_direction), -1.0, 1.0);

  if (sin_angle < 1e-9)
  {
    if (cos_angle < 0.0)
    {
      return current_direction.unitOrthogonal() * M_PI;
    }
    return Eigen::Vector3d::Zero();
  }

  return axis.normalized() * std::atan2(sin_angle, cos_angle);
}

}  // namespace moveit_trace_motion_planner
