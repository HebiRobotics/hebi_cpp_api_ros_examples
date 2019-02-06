#include "arm_kinematics.hpp"
#include "util/grav_comp.hpp"

namespace hebi {
namespace arm {

ArmKinematics::ArmKinematics(const hebi::robot_model::RobotModel& model)
  : model_(model), masses_(model.getFrameCount(HebiFrameTypeCenterOfMass)) {
  // Update the masses
  model_.getMasses(masses_);
}
  
void ArmKinematics::setJointLimits(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions) {
  // Reset:
  use_joint_limits_ = false;
  min_positions_.resize(0);
  max_positions_.resize(0);

  size_t expected_size = model_.getDoFCount();
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

// Return the joint angles to move to a given xyz location
Eigen::VectorXd ArmKinematics::solveIK(
  const Eigen::VectorXd& initial_positions,
  const Eigen::Vector3d& target_xyz) const {
  // NOTE: may want to customize the IK here!

  // TODO: smartly handle exceptions?
  Eigen::VectorXd ik_result_joint_angles(initial_positions.size());
  if (use_joint_limits_) {  
    model_.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::JointLimitConstraint(min_positions_, max_positions_)
    );
  } else {
    model_.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz)
    );
  }
  return ik_result_joint_angles;
}

// Return the joint angles to move to a given xyz location
Eigen::VectorXd ArmKinematics::solveIK(
  const Eigen::VectorXd& initial_positions,
  const Eigen::Vector3d& target_xyz, 
  const Eigen::Vector3d& end_tip) const {
  // NOTE: may want to customize the IK here!

  // TODO: smartly handle exceptions?
  Eigen::VectorXd ik_result_joint_angles(initial_positions.size());
  if (use_joint_limits_) {
    model_.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::EndEffectorTipAxisObjective(end_tip),
      robot_model::JointLimitConstraint(min_positions_, max_positions_)
    );
  } else {
    model_.solveIK(
      initial_positions,
      ik_result_joint_angles,
      robot_model::EndEffectorPositionObjective(target_xyz),
      robot_model::EndEffectorTipAxisObjective(end_tip)
    );
  }
  return ik_result_joint_angles;
}

Eigen::Vector3d ArmKinematics::FK(const Eigen::VectorXd& positions) const {
  Eigen::Matrix4d transform;
  model_.getEndEffector(positions, transform);
  return Eigen::Vector3d(transform(0,3), transform(1,3), transform(2,3));
}

Eigen::VectorXd ArmKinematics::gravCompEfforts(const hebi::GroupFeedback& feedback) const {
  return hebi::util::GravityCompensation::getEfforts(model_, masses_, feedback);
}
  
} // namespace arm
} // namespace hebi
