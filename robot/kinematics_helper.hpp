#include "Eigen/Dense"

namespace hebi {
namespace robot_model {
class RobotModel;
} // namespace robot_model
namespace arm {

// A small helper class for computing IK and storing preferences about how this is done.
class KinematicsHelper {

public:
  KinematicsHelper() = default;

  void setJointLimits(
    const robot_model::RobotModel& robot_model,
    const Eigen::VectorXd& min_positions,
    const Eigen::VectorXd& max_positions);

  void clearJointLimits();

  Eigen::Vector3d FK3Dof(
    const robot_model::RobotModel& robot_model,
    const Eigen::VectorXd& positions) const;

  void FK5Dof(
    const robot_model::RobotModel& robot_model,
    const Eigen::VectorXd& positions,
    Eigen::Vector3d& xyz_out,
    Eigen::Vector3d& tip_axis) const;

  void FK6Dof(
    const robot_model::RobotModel& robot_model,
    const Eigen::VectorXd& positions,
    Eigen::Vector3d& xyz_out,
    Eigen::Matrix3d& orientation) const;

  Eigen::VectorXd solveIK3Dof(
    const robot_model::RobotModel& robot_model,
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz) const;

  Eigen::VectorXd solveIK5Dof(
    const robot_model::RobotModel& robot_model,
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz,
    const Eigen::Vector3d& end_tip) const;

  Eigen::VectorXd solveIK6Dof(
    const robot_model::RobotModel& robot_model,
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz,
    const Eigen::Matrix3d& orientation) const;

private:
  bool use_joint_limits_{};
  Eigen::VectorXd min_positions_{};
  Eigen::VectorXd max_positions_{};
};

} // namespace arm
} // namespace hebi
