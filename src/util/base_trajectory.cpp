#include "base_trajectory.hpp"

BaseTrajectory::~BaseTrajectory() {
}

// Updates the Base State by planning a trajectory to a given set of joint
// waypoints.  Uses the current trajectory/state if defined.
// NOTE: this is a wrapper around the more general replan that
// assumes zero end velocity and acceleration.
void BaseTrajectory::replan(
  double t_now,
  const Eigen::MatrixXd& new_positions) {

  int num_joints = new_positions.rows();
  int num_waypoints = new_positions.cols();

  // Unconstrained velocities and accelerations during the path, but set to
  // zero at the end.
  double nan = std::numeric_limits<double>::quiet_NaN();

  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  velocities.setConstant(nan);
  velocities.rightCols<1>() = Eigen::VectorXd::Zero(num_joints);

  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  accelerations.setConstant(nan);
  accelerations.rightCols<1>() = Eigen::VectorXd::Zero(num_joints);

  replan(t_now, new_positions, velocities, accelerations);
}

void BaseTrajectory::getState(
  double t_now, 
  Eigen::VectorXd& positions,
  Eigen::VectorXd& velocities,
  Eigen::VectorXd& accelerations) {

  // (Cap the effective time to the end of the trajectory)
  double t = std::min(t_now - trajectory_start_time_,
                      trajectory_->getDuration());

  trajectory_->getState(t, &positions, &velocities, &accelerations);
}
