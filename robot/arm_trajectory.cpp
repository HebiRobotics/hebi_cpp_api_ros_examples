#include "arm_trajectory.hpp"
#include "hebi_cpp_api/group_feedback.hpp"

namespace hebi {
namespace arm {

ArmTrajectory ArmTrajectory::create(const Eigen::VectorXd& home_position, const GroupFeedback& feedback, double t_now) {
  ArmTrajectory arm_trajectory;

  // Set up initial trajectory
  Eigen::MatrixXd positions(home_position.size(), 1);
  positions.col(0) = home_position;
  arm_trajectory.replan(t_now, feedback, positions);

  return arm_trajectory;
}

void ArmTrajectory::getState(
  double t_now, 
  Eigen::VectorXd& positions,
  Eigen::VectorXd& velocities,
  Eigen::VectorXd& accelerations) {

  if (override_active_command_) {
    auto num_modules = trajectory_->getJointCount();
    positions.resize(num_modules);
    velocities.resize(num_modules);
    accelerations.resize(num_modules);
    for (size_t i = 0; i < num_modules; ++i) {
			positions(i) = std::numeric_limits<double>::quiet_NaN();
      // Note: maybe want "0" velocity instead?
			velocities(i) = std::numeric_limits<float>::quiet_NaN();
			accelerations(i) = 0;
    }
  } else {
    // (Cap the effective time to the end of the trajectory)
    double t = std::min(t_now - trajectory_start_time_,
                        trajectory_->getDuration());

    trajectory_->getState(t, &positions, &velocities, &accelerations);
  }
}

// Updates the Arm State by planning a trajectory to a given set of joint
// waypoints.  Uses the current trajectory/state if defined.
// NOTE: this call assumes feedback is populated.
void ArmTrajectory::replan(
  double t_now,
  const GroupFeedback& feedback,
  const Eigen::MatrixXd& new_positions,
  const Eigen::MatrixXd& new_velocities,
  const Eigen::MatrixXd& new_accelerations,
  const Eigen::VectorXd& times,
  bool ignore_current_trajectory) {

  int num_joints = new_positions.rows();

  // If there is a current trajectory, use the commands as a starting point;
  // if not, replan from current feedback.
  Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);
  if (trajectory_ && !ignore_current_trajectory) {
    // Clip time to end of trajectory
    double t = std::min(t_now - trajectory_start_time_,
                        trajectory_->getDuration());
    trajectory_->getState(t, &curr_pos, &curr_vel, &curr_accel);
  } else {
    curr_pos = feedback.getPosition();
    curr_vel = feedback.getVelocity();
    // (accelerations remain zero)
  }

  int num_waypoints = new_positions.cols() + 1;

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);

  // Initial state
  positions.col(0) = curr_pos;
  velocities.col(0) = curr_vel;
  accelerations.col(0) = curr_accel;

  // Copy new waypoints
  positions.rightCols(num_waypoints - 1) = new_positions;
  velocities.rightCols(num_waypoints - 1) = new_velocities;
  accelerations.rightCols(num_waypoints - 1) = new_accelerations;

  // Get waypoint times
  Eigen::VectorXd trajTime(num_waypoints);
  // If time vector is empty, automatically determine times
  if (times.size() == 0) {
    trajTime = getWaypointTimes(positions, velocities, accelerations);
  } else {
    trajTime(0) = 0;
    trajTime.tail(num_waypoints - 1) = times;
  }

  // Create new trajectory
  trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                  trajTime, positions, &velocities, &accelerations);
  trajectory_start_time_ = t_now;
}

// Updates the Arm State by planning a trajectory to a given set of joint
// waypoints.  Uses the current trajectory/state if defined.
// NOTE: this call assumes feedback is populated.
void ArmTrajectory::replan(
  double t_now,
  const GroupFeedback& feedback,
  const Eigen::MatrixXd& new_positions,
  const Eigen::MatrixXd& new_velocities,
  const Eigen::MatrixXd& new_accelerations,
  bool ignore_current_trajectory) {

  // Call parent function with empty times matrix to trigger auto-time-determination
  VectorXd empty_times(0);
  replan(t_now, feedback, new_positions, new_velocities, new_accelerations, empty_times, ignore_current_trajectory);
}

// Updates the Arm State by planning a trajectory to a given set of joint
// waypoints.  Uses the current trajectory/state if defined.
// NOTE: this call assumes feedback is populated.
// NOTE: this is a wrapper around the more general replan that
// assumes zero end velocity and acceleration.
void ArmTrajectory::replan(
  double t_now,
  const GroupFeedback& feedback,
  const Eigen::MatrixXd& new_positions,
  bool ignore_current_trajectory) {

  int num_joints = new_positions.rows();
  int num_waypoints = new_positions.cols();

  // Unconstrained velocities and accelerations during the path, but set to
  // zero at the end.
  double nan = std::numeric_limits<double>::quiet_NaN();

  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  velocities.setConstant(nan);
  velocities.rightCols<1>().setZero();

  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  accelerations.setConstant(nan);
  accelerations.rightCols<1>().setZero();

  replan(t_now, feedback, new_positions, velocities, accelerations, ignore_current_trajectory);
}
  
void ArmTrajectory::replan(
  double t_now,
  const GroupFeedback& feedback,
  const Eigen::MatrixXd& new_positions,
  const Eigen::VectorXd& times,
  bool ignore_current_trajectory) {

  int num_joints = new_positions.rows();
  int num_waypoints = new_positions.cols();

  // Unconstrained velocities and accelerations during the path, but set to
  // zero at the end.
  double nan = std::numeric_limits<double>::quiet_NaN();

  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  velocities.setConstant(nan);
  velocities.rightCols<1>().setZero();

  Eigen::MatrixXd accelerations(num_joints, num_waypoints);
  accelerations.setConstant(nan);
  accelerations.rightCols<1>().setZero();

  replan(t_now, feedback, new_positions, velocities, accelerations, times, ignore_current_trajectory);
}

void ArmTrajectory::replan(
  double t_now,
  const GroupFeedback& feedback,
  const Eigen::VectorXd& new_positions,
  bool ignore_current_trajectory) {

  int num_joints = new_positions.size();
  int num_waypoints = 1;

  Eigen::MatrixXd new_positions_matrix(num_joints, 1);
  new_positions_matrix.col(0) = new_positions;

  replan(t_now, feedback, new_positions_matrix, ignore_current_trajectory);
}

// Heuristic to get the timing of the waypoints. This function can be
// modified to add custom waypoint timing.
Eigen::VectorXd ArmTrajectory::getWaypointTimes(
  const Eigen::MatrixXd& positions,
  const Eigen::MatrixXd& velocities,
  const Eigen::MatrixXd& accelerations) {

  // TODO: make this configurable!
  double rampTime = 1.2;

  size_t num_waypoints = positions.cols();

  Eigen::VectorXd times(num_waypoints);
  for (size_t i = 0; i < num_waypoints; ++i)
    times[i] = rampTime * (double)i;

  return times;
}
    
void ArmTrajectory::pauseActiveCommand() {
  override_active_command_ = true; 
}

void ArmTrajectory::resumeActiveCommand(double t_now, const GroupFeedback& feedback) {
  replan(t_now, feedback, feedback.getPosition(), true);
  override_active_command_ = false; 
}

} // namespace arm_node
} // namespace hebi
