#pragma once

#include <memory>

#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "lookup.hpp"
#include "trajectory.hpp"

#include "Eigen/Dense"

/**
 * diff_drive.hpp
 *
 * This file provides an implementation in C++ of an diff drive base, such as that on
 * HEBI's "Rosie" kit.
 *
 * The current implementation provides code for commanding smooth motions of the base.
 */

namespace hebi {

// TODO: DiffDriveTrajectory is _almost_ identical to ArmTrajectory.  Maybe combine
// these?

// Note: base trajectory doesn't allow for smooth replanning, because that would be...difficult.  It just
// represents relative motion in (x, y, theta)

class DiffDriveTrajectory {
public:
  static DiffDriveTrajectory create(const Eigen::VectorXd& dest_positions, double t_now);

  void getState(double t_now, 
    Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations);

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

class DiffDrive {
public:
  // Create an diff drive base with the given modules.  Initialize the trajectory planner
  // with the given time.
  static std::unique_ptr<DiffDrive> create(
    const std::vector<std::string>& families,
    const std::vector<std::string>& names,
    const std::string& gains_file,
    double start_time,
    std::string& error_out);

  // This should be called regularly to ensure commands are sent to the robot.  Recommended
  // calling frequency is ~100Hz. Slow or intermittent calls will result in jerky motion.
  bool update(double time);

  // How far through the last commanded trajectory are we?
  double trajectoryPercentComplete(double time);
  // Has the last commanded trajectory been completed?
  bool isTrajectoryComplete(double time);

  GroupFeedback& getLastFeedback() { return feedback_; }
  DiffDriveTrajectory& getTrajectory() { return base_trajectory_; }

  // Reset before commanding a new trajectory.
  void resetStart(Color& color);

  // Replan trajectory for pure rotation
  void startRotateBy(float radians);
  // Replan trajectory for pure translation
  void startMoveForward(float distance);

  void clearColor();

private:
  DiffDrive(std::shared_ptr<Group> group,
    DiffDriveTrajectory base_trajectory,
    double start_time);

  double convertSE2ToWheel();

  /* Declare main kinematic variables */
  static constexpr double wheel_radius_ = 0.0762; // m
  static constexpr double base_radius_ = 0.235; // m (center of omni to origin of base)

  std::shared_ptr<Group> group_;

  GroupFeedback feedback_;
  GroupCommand command_;

  // These are just temporary variables to cache output from
  // Trajectory::getState.
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;
  Eigen::VectorXd start_wheel_pos_;
  Eigen::VectorXd last_wheel_pos_;
  Eigen::VectorXd wheel_vel_;

  DiffDriveTrajectory base_trajectory_;

  double last_time_{-1};

  Color color_;
};

}
