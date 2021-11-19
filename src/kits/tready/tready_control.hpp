#include "robot/treaded_base.hpp" // For constants!

namespace hebi {

struct TreadyInputs {
  // Flippers
  Eigen::Vector4d flippers_;
  bool align_flippers_;
  // Chassis Velocity ("base_motion" in python code)
  double base_x_vel_;
  double base_y_vel_;
  double base_rot_vel_;
};

struct TreadyVelocity {
  // Flippers
  Eigen::Vector4d flippers_;
  // Chassis Velocity
  double base_x_vel_;
  //double base_y_vel_;
  double base_rot_vel_;
};

// TODO: constructors and stuff; flesh out
struct DemoInputs {

  bool has_value() const { return has_value_; }

  bool shouldExit() { return false; }  // TODO
  bool shouldReset() { return false; } // TODO

  TreadyInputs chassis_;

private:
  bool has_value_{false};
};

// TODO: constructors and stuff; flesh out
struct BaseState {
  bool hasActiveTrajectory() const { return false; } // TODO
  bool aligned_flipper_mode_{};
  bool is_aligning_{};
};

// TODO: constructors and stuff; flesh out
struct BaseCommand {
  static BaseCommand alignFlippers(bool align_state) { return BaseCommand(); }
  static BaseCommand setTrajectories(double t_now) { return BaseCommand(); }
};

// Base class for interfacing with tready control code.  Root state machine used for different operating modes to extend
class TreadyControl {
public:
  enum class DemoState { Startup, Homing, Teleop, Stopped, Exit };

  static constexpr float FLIPPER_VEL_SCALE = 1.f;                                 // rad/sec
  static constexpr float SPEED_MAX_LIN = 0.15f;                                   // m/s
  static constexpr float SPEED_MAX_ROT = SPEED_MAX_LIN / TreadedBase::WHEEL_BASE; // rad/s

  // Track the relevant aspects of the base state; this gets updated when these change (e.g., through a message from the
  // base node)
  void updateBaseState(BaseState base_state);

protected:
private:
  TreadyControl(std::function<void(BaseCommand)> base_command_callback)
    : base_command_callback_(base_command_callback) {
    // TODO
    // def __init__(self, mobile_io, base: TreadedBase):
    //    self.mobile_io = mobile_io
    //    self.base = base
  }

  static TreadyVelocity computeVelocities(const TreadyInputs& chassis_inputs);

  bool update(double t_now, DemoInputs demo_input = {});

  void transitionTo(double t_now, DemoState state);

  DemoState state_{DemoState::Startup};
  // TODO: initial value?
  double mobile_last_fbk_t_{};

  // Keep track of the relevant bits of the base's state here.  Should not be relied on for
  // accurate timing/control flow, just UI, as this may not update instantaneously after setting the state.
  BaseState base_state_;
  std::function<void(BaseCommand)> base_command_callback_;
};


} // namespace hebi