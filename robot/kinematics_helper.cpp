#include "kinematics_helper.hpp"

#include "hebi_cpp_api/robot_model.hpp"

namespace hebi {
namespace arm {

void KinematicsHelper::setJointLimits(
  const robot_model::RobotModel& robot_model,
  const Eigen::VectorXd& min_positions,
  const Eigen::VectorXd& max_positions)
{
  // Reset:
  clearJointLimits();

  size_t expected_size = robot_model.getDoFCount();
  if (min_positions.size() != expected_size || max_positions.size() != expected_size)
    return;

  // Any nans?
  for (size_t i = 0; i < expected_size; ++i)
  {
    if (std::isnan(min_positions_[i]) || std::isnan(max_positions_[i]))
      return;
  }

  min_positions_.resize(expected_size);
  max_positions_.resize(expected_size);
  use_joint_limits_ = true;

  for (size_t i = 0; i < expected_size; ++i)
  {
    min_positions_[i] = min_positions[i];
    max_positions_[i] = max_positions[i];
  }
}

void KinematicsHelper::clearJointLimits()
{
  use_joint_limits_ = false;
  min_positions_.resize(0);
  max_positions_.resize(0);
}

Eigen::VectorXd KinematicsHelper::solveIK3Dof(
  const robot_model::RobotModel& robot_model,
  const Eigen::VectorXd& initial_positions,
  const Eigen::Vector3d& target_xyz) const
{
  // NOTE: may want to customize the IK here!
  // TODO: smartly handle exceptions?
  Eigen::VectorXd ik_result_joint_angles(initial_positions.size());
  if (use_joint_limits_) {  
    robot_model.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::JointLimitConstraint(min_positions_, max_positions_)
    );
  } else {
    robot_model.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz)
    );
  }
  return ik_result_joint_angles;
}

Eigen::VectorXd KinematicsHelper::solveIK5Dof(
  const robot_model::RobotModel& robot_model,
  const Eigen::VectorXd& initial_positions,
  const Eigen::Vector3d& target_xyz,
  const Eigen::Vector3d& end_tip) const
{
  // NOTE: may want to customize the IK here!
  // TODO: smartly handle exceptions?
  Eigen::VectorXd ik_result_joint_angles(initial_positions.size());
  if (use_joint_limits_) {
    robot_model.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::EndEffectorTipAxisObjective(end_tip),
      robot_model::JointLimitConstraint(min_positions_, max_positions_)
    );
  } else {
    robot_model.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::EndEffectorTipAxisObjective(end_tip)
    );
  }
  return ik_result_joint_angles;
}
  
Eigen::VectorXd KinematicsHelper::solveIK6Dof(
  const robot_model::RobotModel& robot_model,
  const Eigen::VectorXd& initial_positions,
  const Eigen::Vector3d& target_xyz,
  const Eigen::Matrix3d& orientation) const
{
  // NOTE: may want to customize the IK here!
  // TODO: smartly handle exceptions?
  Eigen::VectorXd ik_result_joint_angles(initial_positions.size());
  if (use_joint_limits_) {
    robot_model.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::EndEffectorSO3Objective(orientation),
      robot_model::JointLimitConstraint(min_positions_, max_positions_)
    );
  } else {
    robot_model.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::EndEffectorSO3Objective(orientation)
    );
  }
  return ik_result_joint_angles;
}

Eigen::Vector3d KinematicsHelper::FK3Dof(
  const robot_model::RobotModel& robot_model,
  const Eigen::VectorXd& positions) const
{
  Eigen::Matrix4d transform;
  robot_model.getEndEffector(positions, transform);
  return Eigen::Vector3d(transform(0,3), transform(1,3), transform(2,3));
}

void KinematicsHelper::FK5Dof(
  const robot_model::RobotModel& robot_model,
  const Eigen::VectorXd& positions,
  Eigen::Vector3d& xyz_out,
  Eigen::Vector3d& tip_axis) const
{
  Eigen::Matrix4d transform;
  robot_model.getEndEffector(positions, transform);
  xyz_out = Eigen::Vector3d(transform(0,3), transform(1,3), transform(2,3));
  tip_axis = Eigen::Vector3d(transform(0,2), transform(1,2), transform(2,2));
}

void KinematicsHelper::FK6Dof(
  const robot_model::RobotModel& robot_model,
  const Eigen::VectorXd& positions,
  Eigen::Vector3d& xyz_out,
  Eigen::Matrix3d& orientation) const
{
  Eigen::Matrix4d transform;
  robot_model.getEndEffector(positions, transform);
  xyz_out = Eigen::Vector3d(transform(0,3), transform(1,3), transform(2,3));
  orientation = transform.block<3,3>(0,0);
}

} // namespace arm
} // namespace hebi
