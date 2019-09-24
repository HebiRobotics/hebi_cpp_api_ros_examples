/**
 * Simple node to open and close the gripper, using feedback
 * from a bluetooth joystick (e.g., PS4) on a ClearPath robot.
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 23 Sep 2019
 */

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"

// Default to open gripper
double gripper_effort_scale = -1;
int last_button_state = 0;
int last_gripper_state = 0;

void updateJoyState(sensor_msgs::Joy joy_msg) {
  int button_state = joy_msg.buttons[0];
  if (button_state != last_button_state && button_state == 1)
  {
    gripper_effort_scale = static_cast<double>(!last_gripper_state) * 2.0 - 1.0;
    last_gripper_state = !last_gripper_state;
  }
  last_button_state = button_state;
}

// Initialize ROS node
int main(int argc, char ** argv) {

  ros::init(argc, argv, "gripper_node");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Get name/family parameters; default to standard values:
  std::string family;
  if (node.hasParam("family") && node.getParam("family", family)) {
    ROS_INFO("Found and successfully read 'family' parameter");
  } else {
    ROS_WARN("Could not find/read 'family' parameter; defaulting to 'HEBI'");
    family = "HEBI";
  }

  std::string name;
  if (node.hasParam("name") && node.getParam("name", name)) {
    ROS_INFO("Found and successfully read 'name' parameter");
  } else {
    ROS_WARN("Could not find/read 'name' parameter; defaulting to 'Spool'");
    name = "Spool";
  }

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

  // Get a group
  std::shared_ptr<hebi::Group> group;
  {
    hebi::Lookup lookup;
    group = lookup.getGroupFromNames({family}, {name}, 10000);
  }

  if (!group) {
    ROS_ERROR("Gripper group not found! Shutting Down...\n");
    return -1;
  }

  // Load the appropriate gains file, and set gains
  {
    hebi::GroupCommand gains_cmd(group->size());
    if (!gains_cmd.readGains(ros::package::getPath(gains_package) + std::string("/") + gains_file))
      ROS_ERROR("Could not load arm gains file!");
    else {
      bool success = false;
      for (size_t i = 0; i < 5; ++i) {
        success = group->sendCommandWithAcknowledgement(gains_cmd);
        if (success)
          break;
      }
      if (!success)
        ROS_ERROR("Could not set gripper gains!");
    }
  }

  hebi::GroupCommand group_command(group -> size());

  // Constants for effort to close grippers
  constexpr double close_effort = -5;
  auto& effort_cmd = group_command[0].actuator().effort();

  /////////////////// Initialize ROS interface ///////////////////

  ros::Subscriber gripper_button_subscriber =
    node.subscribe<sensor_msgs::Joy>("/bluetooth_teleop/joy", 50, &updateJoyState);

  constexpr double command_rate = 100; // Hz
  ros::Rate loop_rate(command_rate);

  /////////////////// Main Loop ///////////////////

  // Main command loop
  while (ros::ok()) {
    effort_cmd.set(close_effort * gripper_effort_scale);
    group->sendCommand(group_command);

    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
