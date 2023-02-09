#pragma once

#include <memory>

#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/trajectory.hpp"

#include "Eigen/Dense"

/**
 * diff_drive.hpp
 *
 * This file provides an implementation in C++ of an diff drive base.
 *
 * The current implementation provides code for commanding smooth motions of the base.
 */

namespace hebi {

// Note: base trajectory doesn't allow for smooth replanning, because that would be...difficult.  This
// just represents the raw motion of the joints (left, right).

template <int wheels> class DiffDriveTrajectory {
public:
  static DiffDriveTrajectory create(const Eigen::VectorXd& dest_positions, double t_now);

  void getState(double t_now, 
    Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations);

  void replanVels(double t_now, const Eigen::VectorXd& times, const Eigen::MatrixXd& velocities);

  void replan(
    double t_now,
    const Eigen::MatrixXd& new_positions,
    const Eigen::MatrixXd& new_velocities,
    const Eigen::MatrixXd& new_accelerations);

  void replan(
    double t_now,
    const Eigen::MatrixXd& new_positions);

  // Heuristic to get the timing of the waypoints. This function can be
  // modified to add custom waypoint timing.
  Eigen::VectorXd getWaypointTimes(
    const Eigen::MatrixXd& positions,
    const Eigen::MatrixXd& velocities,
    const Eigen::MatrixXd& accelerations);

  std::shared_ptr<hebi::trajectory::Trajectory> getTraj() { return trajectory_; }
  double getTrajStartTime() { return trajectory_start_time_; }
  double getTrajEndTime() { return trajectory_start_time_ + trajectory_->getDuration(); }

private:
  // This is private, because we want to ensure the DiffDriveTrajectory is always
  // initialized correctly after creation; use the "create" factory method
  // instead.
  DiffDriveTrajectory() = default;
      
  std::shared_ptr<hebi::trajectory::Trajectory> trajectory_ {};
  double trajectory_start_time_ {};
};

template <int wheels> class DiffDrive {
public:
  // Create an diff drive base with the given modules.  Initialize the trajectory planner
  // with the given time.
  static std::unique_ptr<DiffDrive> create(
    const std::vector<std::string>& families,
    const std::vector<std::string>& names,
    const std::string& gains_file,
    double start_time,
    std::string& error_out);

  void updateOdometry(const Eigen::Vector4d& wheel_vel, double dt);
  // This should be called regularly to ensure commands are sent to the robot.  Recommended
  // calling frequency is ~100Hz. Slow or intermittent calls will result in jerky motion.
  bool update(double time);

  GroupFeedback& getLastFeedback() { return feedback_; }
  DiffDriveTrajectory<wheels>& getTrajectory() { return base_trajectory_; }

  double last_time_{-1};

  // Odometry/system state
  const Eigen::Vector3d& getGlobalPose() { return global_pose_; }
  const Eigen::Vector3d& getGlobalVelocity() { return global_vel_; }
  const Eigen::Vector3d& getLocalVelocity() { return local_vel_; }

  // How far through the last commanded trajectory are we?
  double trajectoryPercentComplete(double time) {
    return std::min((time - base_trajectory_.getTrajStartTime()) / base_trajectory_.getTraj()->getDuration(), 1.0) * 100;
  }

  // Has the last commanded trajectory been completed?
  bool isTrajectoryComplete(double time) {
    return time > base_trajectory_.getTrajEndTime();
  }

  // Set color (usually before commanding a new trajectory)
  void setColor(Color& color) {
    color_ = color;
  }

  void clearColor() {
    Color c(0, 0, 0, 0);
    color_ = c;
  }

  // Replan trajectory for pure rotation
  void startRotateBy(float radians, double current_time);
  // Replan trajectory for pure translation
  void startMoveForward(float distance, double current_time);
  // Replan trajectory for velocity control
  void startVelControl(double dx, double dtheta, double time);

private:
  DiffDrive(std::shared_ptr<Group> group,
    DiffDriveTrajectory<wheels> base_trajectory,
    const GroupFeedback& feedback,
    double start_time);

  /* Declare main kinematic variables */
  static constexpr double wheel_radius_ = 0.13; // m
  static constexpr double base_radius_ = 0.235; // m (half of distance between center of diff drive wheels)

  std::shared_ptr<Group> group_;

  GroupFeedback feedback_;
  GroupCommand command_;

  // These are just temporary variables to cache output from
  // Trajectory::getState.
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;
  Eigen::VectorXd start_wheel_pos_;

  DiffDriveTrajectory<wheels> base_trajectory_;

  // Track odometry; these are updated every time `update` is called.
  // x/y/theta global pose and velocity
  Eigen::Vector3d global_pose_{0, 0, 0};
  Eigen::Vector3d global_vel_{0, 0, 0};
  // Also, local velocity.
  Eigen::Vector3d local_vel_{0, 0, 0};

  Color color_;
};

}

#include "diff_drive.ipp"