#include "tready_control.hpp"

namespace hebi {

TreadyVelocity TreadyControl::computeVelocities(const TreadyInputs& chassis_inputs) {
  // Flipper Control
  auto flip1 = chassis_inputs.flippers_[0];
  auto flip2 = chassis_inputs.flippers_[1];
  auto flip3 = chassis_inputs.flippers_[2];
  auto flip4 = chassis_inputs.flippers_[3];

  TreadyVelocity res;
  auto sign = [](const double& a) { return a >= 0 ? 1.f : -1.f; };

  if (chassis_inputs.align_flippers_) {
    res.flippers_[0] =
        std::max(std::abs(flip1), std::abs(flip2)) * sign(flip1 + flip2) * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[1] = -res.flippers_[0];
    res.flippers_[2] =
        std::max(std::abs(flip3), std::abs(flip4)) * sign(flip3 + flip4) * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[3] = -res.flippers_[2];
  } else {
    res.flippers_[0] = flip1 * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[1] = -flip2 * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[2] = flip3 * TreadyControl::FLIPPER_VEL_SCALE;
    res.flippers_[3] = -flip4 * TreadyControl::FLIPPER_VEL_SCALE;
  }

  // Mobile Base Control
  res.base_x_vel_ = TreadyControl::SPEED_MAX_LIN * chassis_inputs.base_x_vel_;
  // Probably don't need this? TreadyVelocity just has no y value?
  //res.base_y_vel_ = 0.;
  res.base_rot_vel_ = TreadyControl::SPEED_MAX_ROT * chassis_inputs.base_rot_vel_;

  return res;
}

bool TreadyControl::update(double t_now, DemoInputs demo_input) {
  // Note: the base updates in it's own node...
  // TODO
  // self.base.update(t_now)
  // self.base.send()

  if (state_ == DemoState::Exit)
    return false;

  if (demo_input.has_value()) {
    if (t_now - mobile_last_fbk_t_ > 1.0) {
      // print("mobileIO timeout, disabling motion");
      transitionTo(t_now, DemoState::Stopped);
    }
    return true;
  } else {
    mobile_last_fbk_t_ = t_now;

    if (demo_input.shouldExit())
      transitionTo(t_now, DemoState::Exit);
    else if (demo_input.shouldReset())
      transitionTo(t_now, DemoState::Homing);

    if (state_ == DemoState::Stopped) {
      mobile_last_fbk_t_ = t_now;
      transitionTo(t_now, DemoState::Teleop);
      return true;
    } else if (state_ == DemoState::Homing) {
      if (!base_state_.hasActiveTrajectory())
        transitionTo(t_now, DemoState::Teleop);
      return true;
    }

    else if (state_ == DemoState::Teleop) {
      auto desired_flipper_mode = demo_input.chassis_.align_flippers_;
      if (base_state_.aligned_flipper_mode_ != desired_flipper_mode)
        base_command_callback_(BaseCommand::alignFlippers(desired_flipper_mode));
      else if (!base_state_.is_aligning_) {
        // only accept new base commands if flippers are not aligning
        auto vels = computeVelocities(demo_input.chassis_);
        base_command_callback_(BaseCommand::setTrajectories(t_now));
        // TODO:
                            //self.base.set_chassis_vel_trajectory(t_now, self.base.chassis_ramp_time, chassis_vels)
                            //self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)
      }
      return true;
    }

    else if (state_ == DemoState::Startup) {
      transitionTo(t_now, DemoState::Homing);
      return true;
    }
  }
  return false;
}

void TreadyControl::transitionTo(double t_now, DemoState state) {
  // self transitions are noop
  if (state_ == state)
    return;

  if (state == DemoState::Homing) {
    /*
                    print("TRANSITIONING TO HOMING")
                    self.base.set_color('magenta')
                    msg = ('Robot Homing Sequence\n'
                           'Please wait...')
                    set_mobile_io_instructions(self.mobile_io, msg)

                    # build trajectory
                    flipper_home = np.array([-1, 1, 1, -1]) * np.deg2rad(15 + 45)
                    self.base.chassis_traj = None
                    self.base.set_flipper_trajectory(t_now, 5.0, p=flipper_home)
                    */
  } else if (state == DemoState::Teleop) {
    /*
                    print("TRANSITIONING TO TELEOP")
                    self.base.clear_color()
                    msg = ('Robot Ready to Control\n'
                           'B1: Reset\n'
                           'B6: Joined Flipper\n'
                           'B8 - Quit')
                    set_mobile_io_instructions(self.mobile_io, msg, color="green")
                    */
  } else if (state == DemoState::Stopped) {
    /*print("TRANSITIONING TO STOPPED")
    self.base.chassis_traj = None
    self.base.flipper_traj = None
    self.base.set_color('blue')*/
  } else if (state == DemoState::Exit) {
    /*
                  print("TRANSITIONING TO EXIT")
                  self.base.set_color("red")

                  # unset mobileIO control config
                  self.mobile_io.set_button_mode(6, 0)
                  self.mobile_io.set_button_output(1, 0)
                  self.mobile_io.set_button_output(8, 0)
                  set_mobile_io_instructions(self.mobile_io, 'Demo Stopped', color="red")
    */
  }
  state_ = state;
}

} // namespace hebi
