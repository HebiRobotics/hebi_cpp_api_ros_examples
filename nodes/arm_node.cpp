#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <hebi_cpp_api_examples/TargetWaypoints.h>

#include "robot/arm.hpp"
#include "robot_model.hpp"

#include <ros/console.h>
#include <ros/package.h>

namespace hebi {
  namespace ros {

    class ArmNode {
    public:
      ArmNode(arm::Arm& arm)
        : arm_(arm)
      { }

      // "Jog" the target end effector location in (x,y,z) space, replanning
      // smoothly to the new location
      void offsetTargetCallback(geometry_msgs::Point data) {
        
        // Only update if target changes!
        if (data.x == 0 && data.y == 0 && data.z == 0)
          return;

        // Initialize target from feedback as necessary
        if (!isTargetInitialized()) {
          auto pos = arm_.getKinematics().FK(arm_.getLastFeedback().getPosition());
          target_xyz_.x() = pos.x();
          target_xyz_.y() = pos.y();
          target_xyz_.z() = pos.z();
        }

        // Update the target point
        target_xyz_.x() += data.x / 100.0; // Converts to cm
        target_xyz_.y() += data.y / 100.0; // Converts to cm
        target_xyz_.z() += data.z / 100.0; // Converts to cm
         
        Eigen::VectorXd ik_result_joint_angles =
          arm_.getKinematics().solveIK(arm_.getLastFeedback().getPosition(),
          target_xyz_);

        // Replan:
        arm_.getTrajectory().replan(
          ::ros::Time::now().toSec(),
          arm_.getLastFeedback(),
          ik_result_joint_angles);
      }

      // Replan a smooth joint trajectory from the current location through a
      // series of cartesian waypoints.
      void updateCartesianWaypoints(hebi_cpp_api_examples::TargetWaypoints target_waypoints) {
        size_t num_waypoints = target_waypoints.waypoints_vector.size();

        // These are the joint angles that will be added
        Eigen::MatrixXd positions(arm_.size(), num_waypoints);

        // Plan to each subsequent point from the last position
        Eigen::VectorXd last_position = arm_.getLastFeedback().getPosition();

        // Get joint angles to move to each waypoint
        for (size_t i = 0; i < num_waypoints; ++i) {
          const auto& waypoint = target_waypoints.waypoints_vector[i];

          Eigen::Vector3d xyz(waypoint.x, waypoint.y, waypoint.z);

          // Find the joint angles for the next waypoint, starting from the last
          // waypoint
          last_position = arm_.getKinematics().solveIK(last_position, xyz);

          // Save the waypoints
          positions.col(i) = last_position; 
        }

        // Replan:
        arm_.getTrajectory().replan(
          ::ros::Time::now().toSec(),
          arm_.getLastFeedback(),
          positions);
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

    };

  }
}

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_node");
  ros::NodeHandle node;

  /////////////////// Load parameters ///////////////////

  // Get parameters for name/family of modules; default to standard values:
  std::vector<std::string> families;
  if (node.hasParam("families") && node.getParam("families", families)) {
    ROS_INFO("Found and successfully read 'families' parameter");
  } else {
    ROS_INFO("Could not find/read 'families' parameter; defaulting to 'HEBI'");
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

  // Create robot model
  auto model = hebi::robot_model::RobotModel::loadHRDF(ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file);
  hebi::arm::ArmKinematics arm_kinematics(*model);

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
    families,                   // Family
    names,                      // Names
    home_position,              // Home position
    arm_kinematics,             // Kinematics object
    ros::Time::now().toSec());  // Starting time (for trajectory)  

  if (!arm) {
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::ArmNode arm_node(*arm);

  ros::Subscriber key_subscriber =
    node.subscribe<geometry_msgs::Point>("keys/cmd_vel", 50, &hebi::ros::ArmNode::offsetTargetCallback, &arm_node);

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
