// HEBI C++ API components
#include "hebi_cpp_api/group.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
#include "hebi_cpp_api/robot_model.hpp"
#include "hebi_cpp_api/trajectory.hpp"

// Arm API components
#include "end_effector.hpp"
#include "goal.hpp"
#include "kinematics_helper.hpp"

namespace hebi {
namespace arm {

// A high-level abstraction of a robot arm, coordinating kinematics, control,
// and basic motion planning.
//
// Typical usage is:
//
// arm::Params params;
// params.families_ = {"Arm"};
// params.names_ = {"J1_base", "J2_shoulder", "J3_elbow", "J4_wrist1", "J5_wrist2", "J6_wrist3"};
// params.hrdf_file_ = "my_robot.hrdf"
//
// auto arm = Arm::create(time.now(), params);
// arm->loadGains("my_robot_gains.xml");
//
// while(true) {
//   arm->update(time.now());
//   arm->send();
//   if (some_condition)
//     arm->setGoal(target_goal);
// }
//
// (Note -- in an actual application, you would want to verify the return
// values of many of the functions above to ensure proper operation!)
class Arm {

public:

  //////////////////////////////////////////////////////////////////////////////
  // Setup functions
  //////////////////////////////////////////////////////////////////////////////

  // Parameters for creating an arm
  struct Params {
    // The family and names passed to the "lookup" function to find modules
    // Both are required.
    std::vector<std::string> families_;
    std::vector<std::string> names_;
    // How long a command takes effect for on the robot before expiring.
    int command_lifetime_ = 100;
    // Loop rate, in Hz.  This is how fast the arm update loop will nominally
    // run.
    double control_frequency_ = 200.f;

    // The robot description.  Either supply the hrdf_file _or_ the robot_model.
    std::string hrdf_file_;
    std::shared_ptr<robot_model::RobotModel> robot_model_;

    // Optionally, supply an end effector to be controlled by the "aux" state of
    // provided goals.
    std::shared_ptr<EndEffectorBase> end_effector_;
  };

  // Creates an "Arm" object, and puts it into a "weightless" no-goal control
  // mode.
  static std::unique_ptr<Arm> create(double t, const Params& params);

  // Loads gains from the given .xml file, and sends them to the module. Returns
  // false if the gains file could not be found, if these is a mismatch in
  // number of modules, or the modules do not acknowledge receipt of the gains.
  bool loadGains(const std::string& gains_file);

  //////////////////////////////////////////////////////////////////////////////
  // Accessors
  //////////////////////////////////////////////////////////////////////////////

  // Returns the number of modules / DoF in the arm
  size_t size() const { return group_->size(); }

  // Returns the internal group. Not necessary for most use cases.
  const Group& group() const { return *group_; }

  // Returns the internal robot model. Not necessary for most use cases.
  const robot_model::RobotModel& robotModel() const { return *robot_model_; }

  // Returns the currently active internal trajectory. Not necessary for most
  // use cases.
  // Returns 'nullptr' if there is no active trajectory.
  const trajectory::Trajectory* trajectory() const { return trajectory_.get(); }

  // Returns the end effector object, if given. Not necessary for most use
  // cases.
  // Returns 'nullptr' if there is no end effector.
  const EndEffectorBase* endEffector() const { return end_effector_.get(); }

  // Returns the command last computed by update, or an empty command object
  // if "update" has never successfully run. The returned command can be
  // modified as desired before it is sent to the robot with the send function.
  GroupCommand& pendingCommand() { return command_; }

  // Returns the last feedback obtained by update, or an empty feedback object
  // if "update" has never successfully run.
  const GroupFeedback& lastFeedback() const { return feedback_; }

  //////////////////////////////////////////////////////////////////////////////
  // Main loop functions
  //
  // Typical usage:
  //
  // while(true) {
  //   arm->update(time.now());
  //   arm->send();
  // }
  //////////////////////////////////////////////////////////////////////////////

  // Updates feedback and generates the basic command for this timestep.
  // To retrieve the feedback, call `getLastFeedback()` after this call.
  // You can modify the command object after calling this.
  //
  // Returns 'false' on a connection problem; true on success.
  bool update(double t);

  // Sends the command last computed by "update" to the robot arm.  Any user
  // modifications to the command are included.
  bool send();

  //////////////////////////////////////////////////////////////////////////////
  // Goals
  // 
  // A goal is a desired (joint angle) position that the arm should reach, and
  // optionally information about the time it should reach that goal at and the
  // path (position, velocity, and acceleration waypoints) it should take to
  // get there.
  //
  // The default behavior when a goal is set is for the arm to plan and begin
  // executing a smooth motion from its current state to this goal, with an
  // internal heuristic that defines the time at which it will reach the goal.
  // This immediately overrides any previous goal that was set.
  //
  // If there is no "active" goal the arm is set into a mode where it is
  // actively controlled to be approximately weightless, and can be moved around
  // by hand easily.  This is the default state when the arm is created.
  //
  // After reaching the goal, the arm continues to be commanded with the final
  // joint state of the set goal, and is _not_ implicitly returned to a
  // "weightless" mode.
  //
  // A goal may also define "aux" states to be sent to an end effector
  // associated with the arm.  In this case, the end effector states are
  // treated as "step functions", immediately being commanded at the timestamp
  // of the waypoint they are associated with.  An empty "aux" goal or "NaN"
  // defines a "no transition" at the given waypoint.
  //////////////////////////////////////////////////////////////////////////////

  // Set the current goal waypoint(s), immediately replanning to these
  // location(s) and optionally end effector states.
  // Goal is a commanded position / velocity.
  void setGoal(const Goal& goal);

  // Returns the progress (from 0 to 1) of the current goal, per the last
  // update call.
  //
  // If we have reached the goal, progress is "1".  If there is no active goal,
  // or we have just begun, progress is "0".
  double goalProgress() const;

  // Have we reached the goal?  If there is no goal, returns 'false'
  bool atGoal() const { return goalProgress() >= 1.0; }

  // Cancels any active goal, returning to a "weightless" state which does not
  // actively command position or velocity.
  void cancelGoal();

  //////////////////////////////////////////////////////////////////////////////
  // Helper functions for forward and inverse kinematics.
  //////////////////////////////////////////////////////////////////////////////

  // Use the following joint limits when computing IK
  // Affects future calls to solveIK**.
  void setJointLimits(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions) {
    kinematics_helper_.setJointLimits(*robot_model_, min_positions, max_positions);
  }

  // Do not use joint limits when computing IK
  // Affects future calls to solveIK**.
  void clearJointLimits(const Eigen::VectorXd& min_positions, const Eigen::VectorXd& max_positions) {
    kinematics_helper_.clearJointLimits();
  }

  // Get the end effector (x,y,z) location
  Eigen::Vector3d FK3Dof(const Eigen::VectorXd& positions) const {
    return kinematics_helper_.FK3Dof(*robot_model_, positions);
  }

  // Get the end effector (x,y,z) location and direction, represented by a unit-length vector
  void FK5Dof(const Eigen::VectorXd& positions, Eigen::Vector3d& xyz_out, Eigen::Vector3d& tip_axis) const {
    kinematics_helper_.FK5Dof(*robot_model_, positions, xyz_out, tip_axis);
  }

  // Get the end effector (x,y,z) location and orientation (represented by a rotation matrix)
  void FK6Dof(const Eigen::VectorXd& positions, Eigen::Vector3d& xyz_out, Eigen::Matrix3d& orientation) const {
    kinematics_helper_.FK6Dof(*robot_model_, positions, xyz_out, orientation);
  }

  // Return the joint angles to move to a given xyz location
  Eigen::VectorXd solveIK3Dof(
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz) const {
    return kinematics_helper_.solveIK3Dof(*robot_model_, initial_positions, target_xyz);
  }

  // Return the joint angles to move to a given xyz location while
  // pointing a certain direction
  Eigen::VectorXd solveIK5Dof(
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz,
    const Eigen::Vector3d& end_tip) const {
    return kinematics_helper_.solveIK5Dof(*robot_model_, initial_positions, target_xyz, end_tip);
  }

  // Return the joint angles to move to a given xyz location while
  // pointing a certain direction
  Eigen::VectorXd solveIK6Dof(
    const Eigen::VectorXd& initial_positions,
    const Eigen::Vector3d& target_xyz,
    const Eigen::Matrix3d& orientation) const {
    return kinematics_helper_.solveIK6Dof(*robot_model_, initial_positions, target_xyz, orientation);
  }

private:
  // Private arm constructor
  Arm(double t,
      std::shared_ptr<Group> group,
      std::shared_ptr<robot_model::RobotModel> robot_model,
      std::shared_ptr<EndEffectorBase> end_effector = nullptr) :
    last_time_(t),
    group_(group),
    robot_model_(robot_model),
    end_effector_(end_effector),
    pos_(Eigen::VectorXd::Zero(group->size())),
    vel_(Eigen::VectorXd::Zero(group->size())),
    accel_(Eigen::VectorXd::Zero(group->size())),
    feedback_(group->size()),
    command_(group->size()) {}

  double last_time_;
  std::shared_ptr<Group> group_;
  std::shared_ptr<robot_model::RobotModel> robot_model_;
  std::shared_ptr<EndEffectorBase> end_effector_;

  // The joint angle trajectory for reaching the current goal.
  std::shared_ptr<trajectory::Trajectory> trajectory_;
  double trajectory_start_time_{ std::numeric_limits<double>::quiet_NaN() };
  // These are just temporary variables to cache output from
  // Trajectory::getState.
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;
  Eigen::VectorXd accel_;
  // Along with a trajectory, aux states may be set.  These are the last aux
  // state for each timestep in the trajectory:
  Eigen::VectorXd aux_times_;
  Eigen::MatrixXd aux_;
  // Returns the aux state at this point in the trajectory
  Eigen::VectorXd getAux(double t) const;

  // Robot model helpers for FK + IK
  KinematicsHelper kinematics_helper_;

  hebi::GroupFeedback feedback_;
  hebi::GroupCommand command_;
};

} // namespace arm_node
} // namespace hebi
