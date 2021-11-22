/**
 * Node to control Tready's base
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 19 Nov 2021
 */

#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>

#include "robot/treaded_base.hpp"

// Global variable only set for enabling service callbacks in ROS.

hebi::TreadedBase* global_base = nullptr;
bool global_homing = false;

bool homeService(std_srvs::SetBool::Request& flipper_command, std_srvs::SetBool::Response& res) {
  if (global_base->isAligning())
  {
    res.success = false;
    res.message = "Cannot home while aligning!";
    return true;
  }
  global_homing = true;

  global_base->clearChassisTrajectory();

  auto t = ::ros::Time::now().toSec();
  Eigen::VectorXd flipper_home(4);
  double tmp = 60. * M_PI / 180.; // 60 deg -> radians
  flipper_home << -tmp, tmp, tmp, -tmp;
  global_base->setFlipperTrajectory(t, 5.0, &flipper_home, nullptr);

  res.success = true;

  return true;
}

bool alignService(std_srvs::SetBool::Request& flipper_command, std_srvs::SetBool::Response& res) {
  if (global_homing)
  {
    res.success = false;
    res.message = "Cannot align while homing!";
    return true;
  }

  global_base->setAlignedFlipperMode(flipper_command.data);

  res.success = true;

  return true;
}

// Initialize ROS node
int main(int argc, char** argv) {
  ros::init(argc, argv, "treaded_base_node");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Read the package + path for the gains file
  std::string gains_package;
  if (node.hasParam("gains_package") && node.getParam("gains_package", gains_package)) {
    ROS_INFO("Found and successfully read 'gains_package' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'gains_package' parameter; aborting!");
    return -1;
  }
  std::string gains_file;
  if (node.hasParam("gains_file") && node.getParam("gains_file", gains_file)) {
    ROS_INFO("Found and successfully read 'gains_file' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'gains_file' parameter; aborting!");
    return -1;
  }
  std::string gains_path = ros::package::getPath(gains_package) + std::string("/") + gains_file;

  std::unique_ptr<hebi::TreadedBase> base;
  {
    hebi::Lookup lookup;
    std::string error;
    auto t_start = ::ros::Time::now().toSec();
    auto res = hebi::TreadedBase::create(lookup, "Tready", gains_path, t_start);
    base = std::move(std::get<0>(res));
    error = std::move(std::get<1>(res));
    if (!base) {
      ROS_ERROR("%s", error.c_str());
      return -1;
    }
  }
  global_base = base.get(); 

  /////////////////// Initialize ROS interface ///////////////////

  ros::Subscriber base_vel_subscriber =
      node.subscribe<geometry_msgs::Twist>("cmd_vel", 10, [&](const geometry_msgs::TwistConstPtr& cmd) {
        // Ignore input while aligning or homing flippers!
        ROS_INFO("Got Command Vel");
        if (global_homing || global_base->isAligning())
          return;

        Eigen::VectorXd vels(3);
        vels << cmd->linear.x, 0., cmd->angular.z;
        auto t = ::ros::Time::now().toSec();
        base->setChassisVelTrajectory(t, base->chassisRampTime(), vels);
      });
  ros::Subscriber flipper_vel_subscriber = node.subscribe<std_msgs::Float64MultiArray>(
      "flipper_vel", 10, [&](const std_msgs::Float64MultiArrayConstPtr& velocity_cmd) {
        // Ignore input while aligning or homing flippers!
        if (global_homing || global_base->isAligning())
          return;

        // TODO
        // self.base.set_flipper_trajectory(t_now, self.base.flipper_ramp_time, v=flipper_vels)
      });
  ros::Subscriber color_subscriber =
      node.subscribe<std_msgs::ColorRGBA>("color", 10, [&](const std_msgs::ColorRGBAConstPtr& color_cmd) {
        base->setColor({(uint8_t)(color_cmd->r * 255), (uint8_t)(color_cmd->g * 255), (uint8_t)(color_cmd->b * 255),
                        (uint8_t)(color_cmd->a * 255)});
      });

  // ros::Service
  auto home_flippers = node.advertiseService("home_flippers", &homeService);
  auto align_flippers = node.advertiseService("align_flippers", &alignService);

  /////////////////// Main Loop ///////////////////

  // Main command loop
  while (ros::ok()) {
    auto t = ::ros::Time::now().toSec();
    base->update(t);
    if (!base->hasActiveTrajectory())
      global_homing = false;
    base->send();
    ros::spinOnce();
  }

  return 0;
}