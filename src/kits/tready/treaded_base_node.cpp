/**
 * Node to control Tready's base
 *
 * @author Matthew Tesch < matt @ hebirobotics.com >
 * @since 19 Nov 2021
 */

#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
 
#include "robot/treaded_base.hpp"

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

  /////////////////// Main Loop ///////////////////

  // Main command loop
  while (ros::ok()) {
    auto t = ::ros::Time::now().toSec();
    base->update(t);
    base->send();
    ros::spinOnce();
  }

  return 0;
}