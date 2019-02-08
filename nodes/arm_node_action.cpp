#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <hebi_cpp_api_examples/TargetWaypoints.h>

#include <hebi_cpp_api_examples/ArmMotionAction.h>

#include "actionlib/server/simple_action_server.h"

#include "robot/arm.hpp"
#include "robot_model.hpp"
#include "group_command.hpp"

#include <ros/console.h>
#include <ros/package.h>

namespace hebi {
namespace ros {

class ArmNode {
public:
  ArmNode(arm::Arm& arm)
    : arm_(arm)
  {
  }

  void startArmMotion(const hebi_cpp_api_examples::ArmMotionGoalConstPtr& goal)
  {

    // Replan a smooth joint trajectory from the current location through a
    // series of cartesian waypoints.
    // TODO: use a single struct instead of 6 single vectors of the same length;
    // but how do we do hierarchial actions?
    size_t num_waypoints = goal->x.size();

    // These are the joint angles that will be added
    Eigen::MatrixXd positions(arm_.size(), num_waypoints);

    // Plan to each subsequent point from the last position
    // Eigen::VectorXd last_position = arm_.getLastFeedback().getPosition();
    Eigen::VectorXd last_position = arm_.getLastFeedback().getPositionCommand();

    // Get joint angles to move to each waypoint
    for (size_t i = 0; i < num_waypoints; ++i) {

      Eigen::Vector3d xyz;
      Eigen::Vector3d end_tip;

      // Special homing values...
      if (goal->x[i] == 100 && goal->y[i] == 100 && goal->z[i] == 100) {
        xyz = arm_.getHomePositionXYZ();
        end_tip << 1, 0, 0;
      } 

      else if (goal->x[i] == 101 && goal->y[i] == 101 && goal->z[i] == 101) {
        xyz = arm_.getHomePositionXYZ();
        end_tip << 0, 0, -1;
      } 

      else {
        xyz << goal->x[i], goal->y[i], goal->z[i];
        end_tip << goal->tipx[i], goal->tipy[i], goal->tipz[i];
      }

      // Find the joint angles for the next waypoint, starting from the last
      // waypoint
      last_position = arm_.getKinematics().solveIK(last_position, xyz, end_tip);

      // Save the waypoints
      positions.col(i) = last_position; 
    }

    // Replan:
    arm_.getTrajectory().replan(
      ::ros::Time::now().toSec(),
      arm_.getLastFeedback(),
      positions);
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    arm_.setColor(color);

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);
    bool success = true;

    ROS_INFO("Executing arm motion action");

    hebi_cpp_api_examples::ArmMotionFeedback feedback;

    while (true) {
      if (action_server_->isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Preempted arm motion");
        action_server_->setPreempted();
        success = false;
        break;
      }

      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      auto& arm_traj = arm_.getTrajectory();
      feedback.percent_complete = arm_.trajectoryPercentComplete(t);
      action_server_->publishFeedback(feedback);
 
      if (arm_.isTrajectoryComplete(t)) {
        ROS_INFO("COMPLETE");
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }
    
    arm_.clearColor();

    // publish when the arm is done with a motion
    ROS_INFO("Completed arm motion action");
    hebi_cpp_api_examples::ArmMotionResult result;
    result.success = success;
    action_server_->setSucceeded(result);
  }

  void setActionServer(actionlib::SimpleActionServer<hebi_cpp_api_examples::ArmMotionAction>* action_server) {
    action_server_ = action_server;
  }
  
private:
  arm::Arm& arm_;

  actionlib::SimpleActionServer<hebi_cpp_api_examples::ArmMotionAction>* action_server_ {nullptr};

};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_node_action");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Get parameters for name/family of modules; default to standard values:
  std::vector<std::string> families;
  if (node.hasParam("families") && node.getParam("families", families)) {
    ROS_INFO("Found and successfully read 'families' parameter");
  } else {
    ROS_WARN("Could not find/read 'families' parameter; defaulting to 'HEBI'");
    families = {"HEBI"};
  }

  std::vector<std::string> names;
  if (node.hasParam("names") && node.getParam("names", names)) {
    ROS_INFO("Found and successfully read 'names' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'names' parameter; aborting!");
    return -1;
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

  // Read the package + path for the hrdf file
  std::string hrdf_package;
  if (node.hasParam("hrdf_package") && node.getParam("hrdf_package", hrdf_package)) {
    ROS_INFO("Found and successfully read 'hrdf_package' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'hrdf_package' parameter; aborting!");
    return -1;
  }
  std::string hrdf_file;
  if (node.hasParam("hrdf_file") && node.getParam("hrdf_file", hrdf_file)) {
    ROS_INFO("Found and successfully read 'hrdf_file' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'hrdf_file' parameter; aborting!");
    return -1;
  }

  // Get the "home" position for the arm
  std::vector<double> home_position_vector;
  if (node.hasParam("home_position") && node.getParam("home_position", home_position_vector)) {
    ROS_INFO("Found and successfully read 'home_position' parameter");
  } else {
    ROS_WARN("Could not find/read 'home_position' parameter; defaulting to all zeros!");
  }

  /////////////////// Initialize arm ///////////////////

  // Load robot model
  auto model = hebi::robot_model::RobotModel::loadHRDF(ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file);
  hebi::arm::ArmKinematics arm_kinematics(*model);

  // Get the home position, defaulting to (nearly) zero
  Eigen::VectorXd home_position(model->getDoFCount());
  if (home_position_vector.empty()) {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = 0.01; // Avoid common singularities by being slightly off from zero
    }
  } else if (home_position_vector.size() != model->getDoFCount()) {
    ROS_ERROR("'home_position' parameter not the same length as HRDF file's number of DoF! Aborting!");
    return -1;
  } else {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = home_position_vector[i];
    }
  }

  // Create arm and plan initial trajectory
  auto arm = hebi::arm::Arm::createArm(
    families,                  // Family
    names,                     // Names
    home_position,             // Home position
    arm_kinematics,            // Kinematics object
    ros::Time::now().toSec()); // Starting time (for trajectory)  

  if (!arm) {
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  // Load the appropriate gains file
  {
    hebi::GroupCommand gains_cmd(arm -> size());
    if (!gains_cmd.readGains(ros::package::getPath(gains_package) + std::string("/") + gains_file))
      ROS_ERROR("Could not load arm gains file!");
    else
    {
      bool success = false;
      for (size_t i = 0; i < 5; ++i)
      {
        success = arm->getGroup()->sendCommandWithAcknowledgement(gains_cmd);
        if (success)
          break;
      }
      if (!success)
        ROS_ERROR("Could not set arm gains!");
    }
  }

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::ArmNode arm_node(*arm);

  // Action server for arm motions
  actionlib::SimpleActionServer<hebi_cpp_api_examples::ArmMotionAction> arm_motion_action(
    node, "motion",
    boost::bind(&hebi::ros::ArmNode::startArmMotion, &arm_node, _1), false);

  arm_node.setActionServer(&arm_motion_action);

  arm_motion_action.start();

  /////////////////// Main Loop ///////////////////

  double t_now;

  // Main command loop
  while (ros::ok()) {

    auto t = ros::Time::now().toSec();

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update(t))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
