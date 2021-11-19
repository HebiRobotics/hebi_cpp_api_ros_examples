#include "treaded_base.hpp"

#include <thread>

namespace hebi {

namespace {

Eigen::MatrixXd GetWheelToChassisVel() {
  Eigen::MatrixXd res(3, 4);
  double wr = TreadedBase::WHEEL_RADIUS / (TreadedBase::WHEEL_BASE / 2.);
  // clang-format off
  res << TreadedBase::WHEEL_RADIUS, -TreadedBase::WHEEL_RADIUS, TreadedBase::WHEEL_RADIUS, -TreadedBase::WHEEL_RADIUS,
         0, 0, 0, 0,
         wr, wr, wr, wr;
  // clang-format on
  return res;
}

Eigen::MatrixXd GetChassisToWheelVel() {
  Eigen::MatrixXd res(4, 3);
  // clang-format off
  res << 1. / TreadedBase::WHEEL_RADIUS, 0, TreadedBase::WHEEL_BASE / (2. * TreadedBase::WHEEL_RADIUS),
        -1. / TreadedBase::WHEEL_RADIUS, 0, TreadedBase::WHEEL_BASE / (2. * TreadedBase::WHEEL_RADIUS),
         1. / TreadedBase::WHEEL_RADIUS, 0, TreadedBase::WHEEL_BASE / (2. * TreadedBase::WHEEL_RADIUS),
        -1. / TreadedBase::WHEEL_RADIUS, 0, TreadedBase::WHEEL_BASE / (2. * TreadedBase::WHEEL_RADIUS);
  // clang-format on
  return res;
}

} // namespace

const Eigen::MatrixXd TreadedBase::WHEEL_TO_CHASSIS_VEL = GetWheelToChassisVel();
const Eigen::MatrixXd TreadedBase::CHASSIS_TO_WHEEL_VEL = GetChassisToWheelVel();

std::pair<std::unique_ptr<TreadedBase>, std::string> TreadedBase::create(hebi::Lookup& lookup,
                                                                         const std::string& family) {
  std::vector<std::string> names(8); // 1-4 flipper, 5-8 wheels
  for (int i = 0; i < 4; ++i) {
    names[i] = "T" + std::to_string(i + 1) + "_J1_flipper";
    names[4 + i] = "T" + std::to_string(i + 1) + "_J2_track";
  }

  // Create base group
  auto group = lookup.getGroupFromNames({family}, names);
  if (!group) {
    return {nullptr, "Could not find group on network!"};
  }

  // Try to load gains:
  hebi::GroupCommand gains_command(group->size());
  if (!gains_command.readGains("gains/r-tready-gains.xml"))
    return {nullptr, "Could not read gains from file; check gains are in correct relative path."};

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Send gains multiple times
  bool success = false;
  for (int i = 0; i < 3; ++i) {
    success = group->sendCommandWithAcknowledgement(gains_command);
    if (success)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!success)
    return {nullptr, "Could not send gains to modules; check network connection."};

  // Try to get initial feedback:
  std::unique_ptr<hebi::GroupFeedback> init_feedback(new hebi::GroupFeedback(group->size()));

  success = false;
  for (int i = 0; i < 3; ++i) {
    success = group->getNextFeedback(*init_feedback);
    if (success)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!success)
    return {nullptr, "Could not get feedback from modules; check network connection."};

  std::unique_ptr<TreadedBase> base(new TreadedBase(group, std::move(init_feedback)));
  return {std::move(base), ""};
}

bool TreadedBase::hasActiveTrajectory() const {
  if (chassis_traj_ && t_prev_ < chassis_traj_->getEndTime())
    return true;
  if (flipper_traj_ && t_prev_ < flipper_traj_->getEndTime())
    return true;
  return false;
}

bool TreadedBase::isAligning() const {
  auto active_align = is_aligning_ && hasActiveTrajectory();
  return active_align && !flippersAligned();
}

void TreadedBase::setAlignedFlipperMode(bool value) {
  if (value == aligned_flipper_mode_)
    return;
  else if (value)
    alignFlippers();
  else
    unlockFlippers();
}

Eigen::VectorXd TreadedBase::alignedFlipperPosition() const {
  auto p0 = flipperFeedback(0).actuator().positionCommand().get();
  auto p1 = flipperFeedback(1).actuator().positionCommand().get();
  auto p2 = flipperFeedback(2).actuator().positionCommand().get();
  auto p3 = flipperFeedback(3).actuator().positionCommand().get();
  auto front_mean = p0 * 0.5 - p1 * 0.5;
  auto back_mean = p2 * 0.5 - p3 * 0.5;
  Eigen::VectorXd res(4);
  res[0] = front_mean;
  res[1] = -front_mean;
  res[2] = back_mean;
  res[3] = -back_mean;
  return res;
}

void TreadedBase::update(double t_now) {
  auto dt = t_now - t_prev_;

  bool got_feedback = group_->getNextFeedback(*fbk_);
  if (!flipper_traj_ && !chassis_traj_) {
    // No trajectories, zeroing velocity
    cmd_.setVelocity(Eigen::VectorXd::Zero(cmd_.size()));
  } else {
    if (chassis_traj_) {
      // chassis update
      auto t = std::min(t_now, chassis_traj_->getEndTime());
      Eigen::VectorXd chassis_vel;
      chassis_traj_->getState(t, nullptr, &chassis_vel, nullptr);
      Eigen::VectorXd wheel_vel = CHASSIS_TO_WHEEL_VEL * chassis_vel;
      setWheelVelocityCommand(wheel_vel);
      incrementWheelPositionCommand(wheel_vel * dt);
    }
    if (flipper_traj_) {
      // flipper update
      auto t = std::min(t_now, flipper_traj_->getEndTime());
      Eigen::VectorXd pos, vel;
      flipper_traj_->getState(t, &pos, &vel, nullptr);
      setFlipperVelocityCommand(vel);
      if (aligned_flipper_mode_) {
        if (!flippersAligned()) {
          setFlipperPositionCommand(pos);
        } else {
          setFlipperPositionCommand(alignedFlipperPosition() + vel * dt);
        }
      } else {
        incrementFlipperPositionCommand(vel * dt);
      }
    }
  }
  t_prev_ = t_now;
}

void TreadedBase::setFlipperTrajectory(double t_now, double ramp_time, Eigen::VectorXd* p, Eigen::VectorXd* v) {
  // This is set to true after this call, if the trajectory is an aligning one
  // Otherwise we want it set to False, so clear it now
  is_aligning_ = false;

  Eigen::VectorXd times;
  times << t_now, t_now + ramp_time;
  Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(4, 2);
  Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(4, 2);
  Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(4, 2);
  if (flipper_traj_) {
    auto t = std::min(t_now, flipper_traj_->getEndTime());
    Eigen::VectorXd p_tmp, v_tmp, a_tmp;
    flipper_traj_->getState(t, &p_tmp, &v_tmp, &a_tmp);
    positions.col(0) = p_tmp;
    velocities.col(0) = v_tmp;
    accelerations.col(0) = a_tmp;
  } else {
    for (int row = 0; row < 4; ++row) {
      const auto& ff = flipperFeedback(row);
      positions(row, 0) = ff.actuator().position().get();
      velocities(row, 0) = ff.actuator().velocity().get();
    }
    accelerations.col(0) = Eigen::VectorXd::Zero(4);
  }
  if (p)
    positions.col(1) = *p;
  else
    positions.col(1) = Eigen::VectorXd::Constant(4, std::numeric_limits<double>::quiet_NaN());
  if (v)
    velocities.col(1) = *v;
  else
    velocities.col(1) = Eigen::VectorXd::Constant(4, std::numeric_limits<double>::quiet_NaN());
  accelerations.col(1) = Eigen::VectorXd::Zero(4);

  flipper_traj_ = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);
}

void TreadedBase::setChassisVelTrajectory(double t_now, double ramp_time, const Eigen::VectorXd& v) {
  Eigen::VectorXd times;
  times << t_now, t_now + ramp_time;

  Eigen::MatrixXd positions = Eigen::MatrixXd::Zero(3, 2);
  Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(3, 2);
  Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(3, 2);

  if (chassis_traj_) {
    auto t = std::min(t_now, chassis_traj_->getEndTime());
    Eigen::VectorXd p_tmp, v_tmp, a_tmp;
    flipper_traj_->getState(t, &p_tmp, &v_tmp, &a_tmp);
    positions.col(0) = p_tmp;
    velocities.col(0) = v_tmp;
    accelerations.col(0) = a_tmp;
  } else {
    positions.col(0) = Eigen::VectorXd::Zero(3);
    velocities.col(0) = WHEEL_TO_CHASSIS_VEL * wheelFeedbackVelocity();
    accelerations.col(0) = Eigen::VectorXd::Zero(3);
  }
  positions.col(1) = Eigen::VectorXd::Constant(3, std::numeric_limits<double>::quiet_NaN());
  velocities.col(1) = v;
  accelerations.col(1) = Eigen::VectorXd::Zero(3);

  chassis_traj_ = hebi::trajectory::Trajectory::createUnconstrainedQp(times, positions, &velocities, &accelerations);
}

const hebi::Feedback& TreadedBase::wheelFeedback(int index) const {
  switch (index) {
    case 0:
    case 1:
    case 2:
    case 3:
      return (*fbk_)[index];
    default:
      throw std::out_of_range("Invalid index for flipper feedback");
  }
}

Eigen::VectorXd TreadedBase::wheelFeedbackVelocity() const {
  Eigen::VectorXd res(4);
  res << (*fbk_)[0].actuator().velocity().get(), (*fbk_)[1].actuator().velocity().get(),
      (*fbk_)[2].actuator().velocity().get(), (*fbk_)[3].actuator().velocity().get();
  return res;
}

const hebi::Feedback& TreadedBase::flipperFeedback(int index) const {
  switch (index) {
    case 0:
    case 1:
    case 2:
    case 3:
      return (*fbk_)[index + 4];
    default:
      throw std::out_of_range("Invalid index for flipper feedback");
  }
}

void TreadedBase::setWheelPositionCommand(const Eigen::VectorXd& pos) {
  if (pos.size() != 4)
    throw std::out_of_range("Invalid size for wheel command");
  for (int i = 0; i < 4; ++i)
    cmd_[i].actuator().position().set(pos[i]);
}
void TreadedBase::incrementWheelPositionCommand(const Eigen::VectorXd& pos_increment) {
  if (pos_increment.size() != 4)
    throw std::out_of_range("Invalid size for wheel command");
  for (int i = 0; i < 4; ++i) {
    auto& tmp = cmd_[i].actuator().position();
    tmp.set(tmp.get() + pos_increment[i]);
  }
}
void TreadedBase::setWheelVelocityCommand(const Eigen::VectorXd& vel) {
  if (vel.size() != 4)
    throw std::out_of_range("Invalid size for wheel command");
  for (int i = 0; i < 4; ++i)
    cmd_[i].actuator().velocity().set(vel[i]);
}

void TreadedBase::setFlipperPositionCommand(const Eigen::VectorXd& pos) {
  if (pos.size() != 4)
    throw std::out_of_range("Invalid size for flipper command");
  for (int i = 0; i < 4; ++i)
    cmd_[i + 4].actuator().position().set(pos[i]);
}
void TreadedBase::incrementFlipperPositionCommand(const Eigen::VectorXd& pos_increment) {
  if (pos_increment.size() != 4)
    throw std::out_of_range("Invalid size for flipper command");
  for (int i = 0; i < 4; ++i) {
    auto& tmp = cmd_[i + 4].actuator().position();
    tmp.set(tmp.get() + pos_increment[i]);
  }
}
void TreadedBase::setFlipperVelocityCommand(const Eigen::VectorXd& vel) {
  if (vel.size() != 4)
    throw std::out_of_range("Invalid size for flipper command");
  for (int i = 0; i < 4; ++i)
    cmd_[i + 4].actuator().velocity().set(vel[i]);
}

void TreadedBase::alignFlippers(double* t_now_tmp) {
  double t_now = t_prev_;
  if (t_now_tmp != nullptr)
    t_now = *t_now_tmp;
  aligned_flipper_mode_ = true;
  chassis_traj_ = nullptr;
  if (!flippersAligned()) {
    auto tmp = alignedFlipperPosition();
    setFlipperTrajectory(t_now, 3.0, &tmp);
    is_aligning_ = true;
  }
}

bool TreadedBase::flippersAlignedFront() const {
  return std::abs(flipperFeedback(0).actuator().positionCommand().get() +
                  flipperFeedback(1).actuator().positionCommand().get()) < 0.01f;
}

bool TreadedBase::flippersAlignedBack() const {
  return std::abs(flipperFeedback(2).actuator().positionCommand().get() +
                  flipperFeedback(3).actuator().positionCommand().get()) < 0.01f;
}

} // namespace hebi