#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/group_feedback.hpp"

namespace hebi {
namespace arm {

class ArmKinematics {
public:
  ArmKinematics(const hebi::robot_model::RobotModel& model);

  void setJointLimits(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions);

  // Return the joint angles to move to a given xyz location
  Eigen::VectorXd solveIK(
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz) const;

  // Return the joint angles to move to a given xyz location while
  // pointing a certain direction
  Eigen::VectorXd solveIK(
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz,
    const Eigen::Vector3d& end_tip) const;

  // Return the joint angles to move to a given xyz location while
  // pointing a certain direction
  Eigen::VectorXd solveIKWithOrientation(
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz,
    const Eigen::Matrix3d& orientation) const;

  Eigen::Vector3d FK(const Eigen::VectorXd& positions) const;
  void FKWithTip(const Eigen::VectorXd& positions, Eigen::Vector3d& xyz_out, Eigen::Vector3d& tip_axis) const;
  void FKWithOrientation(const Eigen::VectorXd& positions, Eigen::Vector3d& xyz_out, Eigen::Matrix3d& orientation) const;

  Eigen::VectorXd gravCompEfforts(const hebi::GroupFeedback& feedback) const;

  const hebi::robot_model::RobotModel& getModel() const { return model_; }

private:

  bool use_joint_limits_{};
  Eigen::VectorXd min_positions_{};
  Eigen::VectorXd max_positions_{};

  const hebi::robot_model::RobotModel& model_;
  Eigen::VectorXd masses_;

};

} // namespace arm
} // namespace hebi
