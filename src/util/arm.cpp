#include "arm.hpp"
#include "hebi_cpp_api/lookup.hpp"
#include "src/util/grav_comp.hpp" // TODO: move to / use from hebi C++ api!

#include <cassert>

namespace hebi {
namespace arm {

std::unique_ptr<Arm> Arm::create(double t, const Arm::Params& params) {

  // Load the HRDF:
  std::shared_ptr<robot_model::RobotModel> robot_model;
  if (params.hrdf_file_.empty())
    robot_model = params.robot_model_;
  else
    robot_model = robot_model::RobotModel::loadHRDF(params.hrdf_file_);

  if (!robot_model)
    return nullptr;

  // Get the group (scope the lookup object so it is destroyed
  // immediately after the lookup operation)
  std::shared_ptr<Group> group;
  {
    Lookup lookup;
    group = lookup.getGroupFromNames(params.families_, params.names_);
  }
  if (!group)
    return nullptr;

  // Check sizes
  if (group->size() != robot_model->getDoFCount())
    return nullptr;

  // Set parameters
  if (!group->setCommandLifetimeMs(params.command_lifetime_))
    return nullptr;
  if (!group->setFeedbackFrequencyHz(params.control_frequency_))
    return nullptr;

  // Try to get feedback -- if we don't get a packet in the first N times,
  // something is wrong
  int num_attempts = 0;

  // We need feedback, so we can plan trajectories if that gets called before the first "update"
  GroupFeedback feedback(group->size());
  while (!group->getNextFeedback(feedback)) {
    if (num_attempts++ > 10) {
      return nullptr;
    }
  } 

  // Note: once ROS moves up to C++14, we can change this to "make_unique".
  return std::unique_ptr<Arm>(new Arm(t, group, std::move(robot_model), params.end_effector_));
}
  
bool Arm::loadGains(const std::string& gains_file)
{
  hebi::GroupCommand gains_cmd(group_->size());
  if (!gains_cmd.readGains(gains_file))
    return false;

  return group_->sendCommandWithAcknowledgement(gains_cmd);
}

bool Arm::update(double t) {
  // Time must be monotonically increasing!
  // except if a simulation has just reset...
  // Don't allow trajectories to execute through a simulation reset
  if (t < last_time_) {
    if (trajectory_) {
      std::cout << "Time has moved backwards, canceling active trajectory." << std::endl;
      cancelGoal();
    }
  }

  last_time_ = t;

  if (!group_->getNextFeedback(feedback_))
    return false;

  // Define aux state so end effector can be updated
  Eigen::VectorXd aux(0);

  // Update command from trajectory
  if (trajectory_) {
    // If we have an active trajectory to our goal, use this.
    // Note -- this applies even if we are past the end of it;
    // we just stay with last state.
    // (trajectory_start_time_ should not be nan here!)
    double t_traj = t - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    trajectory_->getState(t_traj, &pos_, &vel_, &accel_);

    aux = getAux(t_traj);
  } else {
    pos_.setConstant(std::numeric_limits<double>::quiet_NaN());
    vel_.setConstant(std::numeric_limits<double>::quiet_NaN());
    accel_.setConstant(0.0);
  }
  command_.setPosition(pos_);
  command_.setVelocity(vel_);

  // Add grav-comp efforts
  Eigen::VectorXd masses;
  // Note -- this could be done just at arm initialization, but end effector
  // mass/payload may change
  robot_model_->getMasses(masses);
  auto gc_efforts = hebi::util::GravityCompensation::getEfforts(*robot_model_, masses, feedback_);
  command_.setEffort(gc_efforts);

  // TODO: add dynamic-comp efforts

  // Update end effector if one exists
  if (end_effector_ && !end_effector_->update(aux))
    return false;

  return true;
}

bool Arm::send() {
  return group_->sendCommand(command_) && (end_effector_ ? end_effector_->send() : true);
}

// TODO: think about adding customizability, or at least more intelligence for
// the default heuristic.
Eigen::VectorXd getWaypointTimes(
  const Eigen::MatrixXd& positions,
  const Eigen::MatrixXd& velocities,
  const Eigen::MatrixXd& accelerations) {

  double rampTime = 1.2;

  size_t num_waypoints = positions.cols();

  Eigen::VectorXd times(num_waypoints);
  for (size_t i = 0; i < num_waypoints; ++i)
    times[i] = rampTime * (double)i;

  return times;
}

void Arm::setGoal(const Goal& goal) {
  int num_joints = goal.positions().rows();

  // If there is a current trajectory, use the commands as a starting point;
  // if not, replan from current feedback.
  Eigen::VectorXd curr_pos = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_vel = Eigen::VectorXd::Zero(num_joints);
  Eigen::VectorXd curr_accel = Eigen::VectorXd::Zero(num_joints);

  // Replan if these is a current trajectory:
  if (trajectory_) {
    double t_traj = last_time_ - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    trajectory_->getState(t_traj, &curr_pos, &curr_vel, &curr_accel);
  } else {
    curr_pos = feedback_.getPosition();
    curr_vel = feedback_.getVelocity();
    // (accelerations remain zero)
  }

  int num_waypoints = goal.positions().cols() + 1;

  Eigen::MatrixXd positions(num_joints, num_waypoints);
  Eigen::MatrixXd velocities(num_joints, num_waypoints);
  Eigen::MatrixXd accelerations(num_joints, num_waypoints);

  // Initial state
  positions.col(0) = curr_pos;
  velocities.col(0) = curr_vel;
  accelerations.col(0) = curr_accel;

  // Copy new waypoints
  positions.rightCols(num_waypoints - 1) = goal.positions();
  velocities.rightCols(num_waypoints - 1) = goal.velocities();
  accelerations.rightCols(num_waypoints - 1) = goal.accelerations();

  // Get waypoint times
  Eigen::VectorXd waypoint_times(num_waypoints);
  // If time vector is empty, automatically determine times
  if (goal.times().size() == 0) {
    waypoint_times = getWaypointTimes(positions, velocities, accelerations);
  } else {
    waypoint_times(0) = 0;
    waypoint_times.tail(num_waypoints - 1) = goal.times();
  }

  // Create new trajectory
  trajectory_ = hebi::trajectory::Trajectory::createUnconstrainedQp(
                waypoint_times, positions, &velocities, &accelerations);
  trajectory_start_time_ = last_time_;

  // Update aux state:
  if (goal.aux().rows() > 0 && (goal.aux().cols() + 1) == num_waypoints) {
    aux_.resize(goal.aux().rows(), goal.aux().cols() + 1);
    aux_.col(0).setConstant(std::numeric_limits<double>::quiet_NaN());
    aux_.rightCols(num_waypoints - 1) = goal.aux();
    aux_times_ = waypoint_times;
  }
  else {
    // Reset aux states!
    aux_.resize(0, 0);
    aux_times_.resize(0);
  }
}

double Arm::goalProgress() const {
  if (trajectory_) {
    double t_traj = last_time_ - trajectory_start_time_;
    t_traj = std::min(t_traj, trajectory_->getDuration());
    return t_traj / trajectory_->getDuration();
  }
  // No current goal!
  return 0.0;
}

void Arm::cancelGoal() {
  trajectory_ = nullptr;
  trajectory_start_time_ = std::numeric_limits<double>::quiet_NaN();
}

Eigen::VectorXd Arm::getAux(double t) const {
  Eigen::VectorXd res;
  if (aux_times_.size() == 0 || aux_.cols() != aux_times_.size() || aux_.rows() == 0)
    return Eigen::VectorXd();

  // Find the first time
  // TODO: use a tracer for performance here...or at least a std::upper_bound/etc.
  for (int i = aux_times_.size() - 1; i >= 0; --i) {
    if (t >= aux_times_[i]) {
      return aux_.col(i);
    }
  }

  // Note -- should never get here...should always be bigger than the initial t == 0...
  return aux_.col(0);
}

} // namespace arm
} // namespace hebi
