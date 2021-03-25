#pragma once

#include "hebi_cpp_api/trajectory.hpp"

#include "Eigen/Dense"

class BaseTrajectory {
public:
  virtual ~BaseTrajectory();

  void replan(double t_now, const Eigen::MatrixXd& new_positions);
  void getState(double t_now, Eigen::VectorXd& positions, Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations);

  virtual void replan(double t_now, const Eigen::MatrixXd& new_positions, const Eigen::MatrixXd& new_velocities, const Eigen::MatrixXd& new_accelerations) = 0;

  std::shared_ptr<hebi::trajectory::Trajectory> getTraj() { return trajectory_; }
  double getTrajStartTime() { return trajectory_start_time_; }
  double getTrajEndTime() { return trajectory_start_time_ + trajectory_->getDuration(); }

protected:
  BaseTrajectory()=default;

  std::shared_ptr<hebi::trajectory::Trajectory> trajectory_ {};
  double trajectory_start_time_ {};
};
