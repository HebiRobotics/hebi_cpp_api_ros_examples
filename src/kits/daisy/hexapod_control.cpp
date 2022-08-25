/**
 * Control a HEBI Hexapod (Lily) from ROS
 *
 * HEBI Robotics
 * September 2018
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include "robot/leg.hpp"
#include "robot/hexapod.hpp"

#include "hebi_cpp_api/lookup.hpp"


namespace hebi {

// This is the ROS interface to the hexapod, receiving commands from messages
// and setting them on the hexapod object.
class HexapodROSInterface {
public:
  HexapodROSInterface(hebi::Hexapod& hex, ros::Publisher& feedback_publisher)
    : hex_(hex), feedback_publisher_(feedback_publisher) {
    translation_velocity_cmd_.setZero();
    rotation_velocity_cmd_.setZero();
  }

  void updateVelocityCommand(geometry_msgs::Twist vel_cmd) {
    // Limit the actual commanded velocities to reasonable max absolute values:
    constexpr double max_trans_vel = 0.175;
    constexpr double max_rot_vel = 0.4;
    translation_velocity_cmd_(0) = std::min(std::max(-vel_cmd.linear.x, -max_trans_vel), max_trans_vel);
    translation_velocity_cmd_(1) = std::min(std::max(-vel_cmd.linear.y, -max_trans_vel), max_trans_vel);
    translation_velocity_cmd_(2) = std::min(std::max(-vel_cmd.linear.z, -max_trans_vel), max_trans_vel);
    rotation_velocity_cmd_(0) = std::min(std::max(-vel_cmd.angular.x, -max_trans_vel), max_trans_vel);
    rotation_velocity_cmd_(1) = std::min(std::max(-vel_cmd.angular.y, -max_trans_vel), max_trans_vel);
    rotation_velocity_cmd_(2) = std::min(std::max(-vel_cmd.angular.z, -max_trans_vel), max_trans_vel);
  }

  void updateMode(std_msgs::Bool stance_mode) {
    hex_.updateMode(stance_mode.data);
  }

  const Eigen::Vector3d& getTranslationVelocityCmd() {
    return translation_velocity_cmd_;
  }

  const Eigen::Vector3d& getRotationVelocityCmd() {
    return rotation_velocity_cmd_;
  }

  void publishFeedback(const Eigen::VectorXd& positions) {

    feedback_msg_.position.resize(positions.size());
    for (size_t i = 0; i < positions.size(); ++i)
      feedback_msg_.position[i] = positions[i];

    // SHOW: Added vel/effort pub
    feedback_msg_.velocity.resize(velocities.size());
    for (size_t i = 0; i < velocities.size(); ++i)
      feedback_msg_.velocity[i] = velocities[i];

    feedback_msg_.effort.resize(efforts.size());
    for (size_t i = 0; i < efforts.size(); ++i)
      feedback_msg_.effort[i] = efforts[i];

    feedback_publisher_.publish(feedback_msg_);
  }

private:
  hebi::Hexapod& hex_;
 
  Eigen::Vector3d translation_velocity_cmd_;
  Eigen::Vector3d rotation_velocity_cmd_;

  // publish feedback!
  sensor_msgs::JointState feedback_msg_;
  ros::Publisher& feedback_publisher_;
};

} // namespace hebi

int main(int argc, char** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "hexapod_control");
  ros::NodeHandle node;

  /////////////////// Initialize hexapod ///////////////////

  std::string resource_path = ros::package::getPath("hebi_cpp_api_examples");
  resource_path += "/data/hexapod/";

  hebi::HexapodErrors hex_errors;
  hebi::HexapodParameters params;
  if (!params.loadFromFile(resource_path + "/hex_config.xml")) {
    std::cout << "Could not find parameter file!\n";
    std::cout << resource_path + "/hex_config.xml";
    return -1;
  }
  params.resource_path_ = resource_path;

  std::unique_ptr<hebi::Hexapod> hexapod = hebi::Hexapod::create(params, hex_errors);

  // Try a couple of times... 
  while (!hexapod) {
    std::cout << "Looking for robot...\n";
    hexapod = hebi::Hexapod::create(params, hex_errors);
  }
  std::cout << "Found robot -- starting control program.\n";
  if (hexapod && !hexapod->setGains() && !hexapod->setGains()) {
    ROS_WARN("Unable to set hexapod gains, things may behave strangely");
  }

  /////////////////////// Initialize ROS Interface //////////////////////

  // Publish joint state message  
  ros::Publisher feedback_publisher = node.advertise<sensor_msgs::JointState>(
    "joint_feedback", 100);

  hebi::HexapodROSInterface hex_ros_if(*hexapod, feedback_publisher);

  // Subscribe to velocity command and mode toggle messages
  ros::Subscriber vel_cmd_subscriber =
    node.subscribe<geometry_msgs::Twist>("velocity_command", 50, &hebi::HexapodROSInterface::updateVelocityCommand, &hex_ros_if);
  ros::Subscriber stance_mode_subscriber =
    node.subscribe<std_msgs::Bool>("mode_select", 50, &hebi::HexapodROSInterface::updateMode, &hex_ros_if); // True for stance, false for step

  // Controls to send to the robot
  Eigen::VectorXd angles(hebi::Leg::getNumJoints());
  Eigen::VectorXd vels(hebi::Leg::getNumJoints());
  Eigen::VectorXd torques(hebi::Leg::getNumJoints());
  Eigen::MatrixXd foot_forces(3,6); // 3 (xyz) by num legs
  foot_forces.setZero();

  /////////////////// Main Loop ///////////////////

  // Run at 100 Hz:
  ros::Duration period(0.01);

  bool startup = true;
  bool first_run = true;
  std::vector<std::shared_ptr<hebi::trajectory::Trajectory>> startup_trajectories;

  // Timekeeping initialization:
  double startup_seconds = 4.5;
  auto start_time = ros::Time::now().toSec();
  double prev_time;
  double dt = 0;

  // Main command loop
  while (ros::ok()) {

    // Timekeeping:
    auto t = ros::Time::now().toSec();
    auto elapsed = t - start_time;
    dt = t - prev_time;
    prev_time = t;

    // TODO: handle "startup" phase here...

    // Startup phase: smoothly transition!
    if (startup)
    {
      int num_joints = hebi::Leg::getNumJoints();
      // TODO: move all logic inside leg?
      if (first_run)
      {
        // V l in legs, let s_l = start joints and e_l = end joints and t_l =
        // trajectory, with zero vel/accel endpoints.
        for (int i = 0; i < 6; ++i)
        {
          Eigen::VectorXd leg_start = hexapod->getLegFeedback(i);
          Eigen::VectorXd leg_end;
          Eigen::VectorXd leg_vels;
          Eigen::MatrixXd jacobian_ee(0, 0);
          hebi::robot_model::MatrixXdVector jacobian_com;
          hexapod->getLeg(i)->computeState(elapsed, leg_end, leg_vels, jacobian_ee, jacobian_com);
          // TODO: fix! (quick and dirty -- leg mid is hardcoded as offset from leg end)
          Eigen::VectorXd leg_mid = leg_end;
          leg_mid(1) -= 0.3;
          leg_mid(2) -= 0.15;

          // Convert for trajectories
          int num_waypoints = 5;
          Eigen::MatrixXd positions(num_joints, num_waypoints);
          Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
          Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
          Eigen::VectorXd nan_column = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());
          // Is this one of the legs that takes a step first?
          bool step_first = (i == 0 || i == 3 || i == 4);

          // Set positions
          positions.col(0) = leg_start;
          positions.col(1) = step_first ? leg_mid : leg_start;
          positions.col(2) = step_first ? leg_end : leg_start;
          positions.col(3) = step_first ? leg_end : leg_mid;
          positions.col(4) = leg_end;

          velocities.col(1) = nan_column;
          velocities.col(3) = nan_column;
          accelerations.col(1) = nan_column;
          accelerations.col(3) = nan_column;

          Eigen::VectorXd times(num_waypoints);
          double local_start = elapsed;
          double total = startup_seconds - local_start;
          times << local_start,
                   local_start + total * 0.25,
                   local_start + total * 0.5,
                   local_start + total * 0.75,
                   local_start + total;
          startup_trajectories.push_back(hebi::trajectory::Trajectory::createUnconstrainedQp(
            times, positions, &velocities, &accelerations));

        }

        // TODO: move this startup logic outside of t.elapse() call, and then
        // we don't need 'first_run'?
        first_run = false;
      }
     
      // Follow t_l:
      for (int i = 0; i < 6; ++i)
      {
        Eigen::VectorXd v(3);
        Eigen::VectorXd a(3);
        startup_trajectories[i]->getState(elapsed, &angles, &v, &a);
        vels = v;

        Eigen::Vector3d foot_force = foot_forces.block<3,1>(0,i);
        hebi::Leg* curr_leg = hexapod->getLeg(i);

        // Get the Jacobian
        Eigen::MatrixXd jacobian_ee;
        hebi::robot_model::MatrixXdVector jacobian_com;
        curr_leg->computeJacobians(angles, jacobian_ee, jacobian_com);

        Eigen::Vector3d gravity_vec = hexapod->getGravityDirection() * 9.8;
        torques = curr_leg->computeTorques(jacobian_com, jacobian_ee, angles, vels, gravity_vec, /* dynamic_comp_torque,*/ foot_force); // TODO: add dynamic compensation
        // TODO: add actual foot torque for startup?
        // TODO: add vel, torque; test each one!
        hexapod->setCommand(i, &angles, &vels, &torques);
      }
      hexapod->sendCommand();

      hex_ros_if.publishFeedback(hexapod->getLastFeedback());

      // Call any pending callbacks (note -- this may update our planned motion)
      ros::spinOnce();

      period.sleep();

      if (elapsed >= startup_seconds)
        startup = false;
      continue;
    }

    // Optionally slowly ramp up commands over the first few seconds
    double ramp_up_scale = std::min(1.0, elapsed / startup_seconds);

    // Actually control the hexapod:
    hexapod->updateStance(
      hex_ros_if.getTranslationVelocityCmd(),
      hex_ros_if.getRotationVelocityCmd(),
      dt);

    if (hexapod->needToStep())
      hexapod->startStep(elapsed);

    hexapod->updateSteps(elapsed);

    // Calculate how the weight is distributed
    hexapod->computeFootForces(elapsed, foot_forces);

    foot_forces *= ramp_up_scale;

    Eigen::MatrixXd jacobian_ee;
    hebi::robot_model::MatrixXdVector jacobian_com;
    Eigen::VectorXd angles_plus_dt;

    for (int i = 0; i < 6; ++i) {
      hebi::Leg* curr_leg = hexapod->getLeg(i);
      curr_leg->computeState(elapsed, angles, vels, jacobian_ee, jacobian_com);

      // Get torques
      Eigen::Vector3d foot_force = foot_forces.block<3,1>(0,i);
      Eigen::Vector3d gravity_vec = hexapod->getGravityDirection() * 9.8;
      torques = hexapod->getLeg(i)->computeTorques(jacobian_com, jacobian_ee, angles, vels, gravity_vec, /*dynamic_comp_torque,*/ foot_force); // TODO:

      hexapod->setCommand(i, &angles, &vels, &torques);
    }
    hexapod->sendCommand();

    hex_ros_if.publishFeedback(hexapod->getLastFeedback());

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();

    period.sleep();
  }

  return 0;
}
