#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Point.h>

#include <hebi_cpp_api_examples/TargetWaypoints.h>
#include <hebi_cpp_api_examples/ArmMotionAction.h>

#include "group_command.hpp"

#include "robot/arm.hpp"
#include "robot_model.hpp"

namespace hebi {
namespace ros {

class ArmNode {
public:
  ArmNode(arm::Arm& arm) : arm_(arm) { }

  // Replan a smooth joint trajectory from the current location through a
  // series of cartesian waypoints.
  // xyz positions should be a 3xn vector of target positions
  void updateCartesianWaypoints(const Eigen::Matrix3Xd& xyz_positions,
      const Eigen::Matrix3Xd& end_tip_directions) {
    // Data sanity check:
    if (end_tip_directions.cols() > 0 && end_tip_directions.cols() != xyz_positions.cols())
      return;

    // Update stored target position:
    target_xyz_ = xyz_positions.col(xyz_positions.cols() - 1);

    // These are the joint angles that will be added
    auto num_waypoints = xyz_positions.cols();
    Eigen::MatrixXd positions(arm_.size(), num_waypoints);

    // Plan to each subsequent point from the last position
    // (We use the last position command for smoother motion)
    Eigen::VectorXd last_position = arm_.getLastFeedback().getPositionCommand();

    // Get joint angles to move to each waypoint
    for (size_t i = 0; i < num_waypoints; ++i) {

      // Find the joint angles for the next waypoint, starting from the last
      // waypoint, and save into the position vector.
      if (end_tip_directions.cols() > 0) {
        // If we are given tip directions, add these too...
        last_position = arm_.getKinematics().solveIK(last_position, xyz_positions.col(i), end_tip_directions.col(i));
      } else {
        last_position = arm_.getKinematics().solveIK(last_position, xyz_positions.col(i));
      }
      positions.col(i) = last_position; 
    }

    // Replan:
    arm_.getTrajectory().replan(
      ::ros::Time::now().toSec(),
      arm_.getLastFeedback(),
      positions);
  }

  void updateCartesianWaypoints(hebi_cpp_api_examples::TargetWaypoints target_waypoints) {
    action_server_->setAborted();

    // Fill in an Eigen::Matrix3xd with the xyz goal
    size_t num_waypoints = target_waypoints.waypoints_vector.size();
    Eigen::Matrix3Xd xyz_waypoints(3, num_waypoints);
    for (size_t i = 0; i < num_waypoints; ++i) {
      const auto& xyz_waypoint = target_waypoints.waypoints_vector[i];
      xyz_waypoints(0, i) = xyz_waypoint.x;
      xyz_waypoints(1, i) = xyz_waypoint.y;
      xyz_waypoints(2, i) = xyz_waypoint.z;
    }

    // Replan
    Eigen::Matrix3Xd tip_directions(3, 0);
    updateCartesianWaypoints(xyz_waypoints, tip_directions);
  }

  // Set the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void setTargetCallback(geometry_msgs::Point data) {
    action_server_->setAborted();

    // Fill in an Eigen::Matrix3xd with the xyz goal
    Eigen::Matrix3Xd xyz_waypoints(3, 1);
    xyz_waypoints(0, 0) = data.x;
    xyz_waypoints(1, 0) = data.y;
    xyz_waypoints(2, 0) = data.z;

    // Replan
    Eigen::Matrix3Xd tip_directions(3, 0);
    updateCartesianWaypoints(xyz_waypoints, tip_directions);
  }

  // "Jog" the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void offsetTargetCallback(geometry_msgs::Point data) {
    action_server_->setAborted();

    // Only update if target changes!
    if (data.x == 0 && data.y == 0 && data.z == 0)
      return;

    // Initialize target from feedback as necessary
    if (!isTargetInitialized()) {
      auto pos = arm_.getKinematics().FK(arm_.getLastFeedback().getPositionCommand());
      target_xyz_.x() = pos.x();
      target_xyz_.y() = pos.y();
      target_xyz_.z() = pos.z();
    }

    // Fill in an Eigen::Matrix3xd with the xyz goal
    Eigen::Matrix3Xd xyz_waypoints(3, 1);
    xyz_waypoints(0, 0) = target_xyz_.x() + data.x;
    xyz_waypoints(1, 0) = target_xyz_.y() + data.y;
    xyz_waypoints(2, 0) = target_xyz_.z() + data.z;

    // Replan
    Eigen::Matrix3Xd tip_directions(3, 0);
    updateCartesianWaypoints(xyz_waypoints, tip_directions);
  }

  void startArmMotion(const hebi_cpp_api_examples::ArmMotionGoalConstPtr& goal) {
    ROS_INFO("Executing arm motion action");

    // Replan a smooth joint trajectory from the current location through a
    // series of cartesian waypoints.
    // TODO: use a single struct instead of 6 single vectors of the same length;
    // but how do we do hierarchial actions?
    size_t num_waypoints = goal->x.size();

    Eigen::Matrix3Xd xyz_positions(3, num_waypoints);
    Eigen::Matrix3Xd tip_directions(3, num_waypoints);

    // Get joint angles to move to each waypoint
    for (size_t i = 0; i < num_waypoints; ++i) {
      // Special homing values...
      if (goal->x[i] == 100 && goal->y[i] == 100 && goal->z[i] == 100) {
        xyz_positions.col(i) = arm_.getHomePositionXYZ();
        tip_directions(0, i) = 1;
        tip_directions(1, i) = 0;
        tip_directions(2, i) = 0;
      }
      else if (goal->x[i] == 101 && goal->y[i] == 101 && goal->z[i] == 101) {
        xyz_positions.col(i) = arm_.getHomePositionXYZ();
        tip_directions(0, i) = 0;
        tip_directions(1, i) = 0;
        tip_directions(2, i) = -1;
      }
      else {
        xyz_positions(0, i) = goal->x[i];
        xyz_positions(1, i) = goal->y[i];
        xyz_positions(2, i) = goal->z[i];
        tip_directions(0, i) = goal->tipx[i];
        tip_directions(1, i) = goal->tipy[i];
        tip_directions(2, i) = goal->tipz[i];
      }
    }

    updateCartesianWaypoints(xyz_positions, tip_directions);

    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    arm_.setColor(color);

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);

    hebi_cpp_api_examples::ArmMotionFeedback feedback;

    while (true) {
      if (action_server_->isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Arm motion action was preempted");
        arm_.clearColor();
        // Note -- the `startArmMotion` function will not be called until the
        // action server has been preempted here:
        action_server_->setPreempted();
        return;
      }

      if (!action_server_->isActive() || !::ros::ok()) {
        ROS_INFO("Arm motion was cancelled");
        arm_.clearColor();
        return;
      }

      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      auto& arm_traj = arm_.getTrajectory();
      feedback.percent_complete = arm_.trajectoryPercentComplete(t);
      action_server_->publishFeedback(feedback);
 
      if (arm_.isTrajectoryComplete(t)) {
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }
    
    arm_.clearColor();

    // publish when the arm is done with a motion
    ROS_INFO("Completed arm motion action");
    action_server_->setSucceeded();
  }

  void setActionServer(actionlib::SimpleActionServer<hebi_cpp_api_examples::ArmMotionAction>* action_server) {
    action_server_ = action_server;
  }
  
private:
  arm::Arm& arm_;

  // The end effector location that this arm will target (NaN indicates
  // unitialized state, and will be set from feedback during first offset)
  Eigen::Vector3d target_xyz_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  bool isTargetInitialized() {
    return !std::isnan(target_xyz_.x()) ||
           !std::isnan(target_xyz_.y()) ||
           !std::isnan(target_xyz_.z());
  }


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

  // "Jog" the end effector
  ros::Subscriber offset_target_subscriber =
    node.subscribe<geometry_msgs::Point>("offset_target", 50, &hebi::ros::ArmNode::offsetTargetCallback, &arm_node);

  // Explicitly set the end effector's target position
  ros::Subscriber set_target_subscriber =
    node.subscribe<geometry_msgs::Point>("set_target", 50, &hebi::ros::ArmNode::setTargetCallback, &arm_node);

  // Subscribe to lists of (x, y, z) waypoints
  ros::Subscriber waypoint_subscriber =
    node.subscribe<hebi_cpp_api_examples::TargetWaypoints>("cartesian_waypoints", 50, &hebi::ros::ArmNode::updateCartesianWaypoints, &arm_node);

  /////////////////// Main Loop ///////////////////

  // Main command loop
  while (ros::ok()) {
    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update(ros::Time::now().toSec()))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
