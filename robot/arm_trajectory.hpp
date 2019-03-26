#include "hebi_cpp_api/trajectory.hpp"
#include "hebi_cpp_api/group.hpp"

namespace hebi {
namespace arm {

// A structure describing the current smooth trajectory for the arm
class ArmTrajectory {
public:
  static ArmTrajectory create(
    const Eigen::VectorXd& home_position,
    const GroupFeedback& feedback,
    double t_now);

  void getState(
    double t_now,
    Eigen::VectorXd& positions,
    Eigen::VectorXd& velocities,
    Eigen::VectorXd& accelerations);
  
  // Updates the Arm State by planning a trajectory to a given set of joint
  // waypoints.  Uses the current trajectory/state if defined.
  // NOTE: this call assumes feedback is populated.
  void replan(
    double t_now,
    const GroupFeedback& feedback,
    const Eigen::MatrixXd& new_positions,
    const Eigen::MatrixXd& new_velocities,
    const Eigen::MatrixXd& new_accelerations,
    const Eigen::VectorXd& times,
    bool ignore_current_trajectory = false);

  // Updates the Arm State by planning a trajectory to a given set of joint
  // waypoints.  Uses the current trajectory/state if defined.
  // NOTE: this call assumes feedback is populated.
  void replan(
    double t_now,
    const GroupFeedback& feedback,
    const Eigen::MatrixXd& new_positions,
    const Eigen::MatrixXd& new_velocities,
    const Eigen::MatrixXd& new_accelerations,
    bool ignore_current_trajectory = false);

  // Updates the Arm State by planning a trajectory to a given set of joint
  // waypoints.  Uses the current trajectory/state if defined.
  // NOTE: this call assumes feedback is populated.
  // NOTE: this is a wrapper around the more general replan that
  // assumes zero end velocity and acceleration, and unconstrained
  // intermediate velocity and acceleration.
  void replan(
    double t_now,
    const GroupFeedback& feedback,
    const Eigen::MatrixXd& new_positions,
    bool ignore_current_trajectory = false);

  // Single-waypoint version of replan
  // NOTE: this call assumes feedback is populated.
  // NOTE: this is a wrapper around the more general replan that assumes a
  // single waypoint, and zero end velocity and acceleration.
  void replan(
    double t_now,
    const GroupFeedback& feedback,
    const Eigen::VectorXd& new_positions,
    bool ignore_current_trajectory = false);

  // Heuristic to get the timing of the waypoints. This function can be
  // modified to add custom waypoint timing.
  Eigen::VectorXd getWaypointTimes(
    const Eigen::MatrixXd& new_positions,
    const Eigen::MatrixXd& new_velocities,
    const Eigen::MatrixXd& new_accelerations);
  
  std::shared_ptr<hebi::trajectory::Trajectory> getTraj() { return trajectory_; }
  double getTrajStartTime() { return trajectory_start_time_; }
  double getTrajEndTime() { return trajectory_start_time_ + trajectory_->getDuration(); }

  // Pause the active command/trajectory to go into a "grav comp" mode
  // where the robot can be moved around.
  void pauseActiveCommand();

  // Resume control to a specific point
  // NOTE: This resets the trajectory to command to the given point
  void resumeActiveCommand(double t_now, const GroupFeedback& feedback);

private:
  // This is private, because we want to ensure the ArmTrajectory is always
  // initialized correctly after creation; use the "create" factory method
  // instead.
  ArmTrajectory() = default;

  // These should _always_ be valid
  std::shared_ptr<hebi::trajectory::Trajectory> trajectory_ {};
  double trajectory_start_time_ {};

  bool override_active_command_{};
};

} // namespace arm
} // namespace hebi
