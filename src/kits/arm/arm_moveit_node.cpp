#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include "hebi_cpp_api/robot_model.hpp"

#include "src/util/arm.hpp"

namespace hebi {
namespace ros {

// This is a node that translates between ROS commands and the hebi Arm object.
// It is designed to use the same interface that MoveIt expects, namely an
// action server for:
// FollowJointTrajectory (on hebi_arm_controller/follow_joint_trajectory)
class MoveItArmNode {
public:
  MoveItArmNode(arm::Arm& arm, ::ros::NodeHandle& node, std::vector<std::string> moveit_joints)
    : arm_(arm),
      action_server_(node, "hebi_arm_controller/follow_joint_trajectory", boost::bind(&MoveItArmNode::followJointTrajectory, this, _1), false),
      joint_state_publisher_(node.advertise<sensor_msgs::JointState>("joint_states", 100)) {

    // The "moveit_joints" here are loaded from a rosparam, and should match those in the
    // moveit_config for the arm.
    auto num_modules = moveit_joints.size();
    joint_state_message_.name = moveit_joints;
    joint_state_message_.position.resize(num_modules, 0 );
    joint_state_message_.velocity.resize(num_modules, 0 );
    joint_state_message_.effort.resize(num_modules, 0 );

    action_server_.start();
  }

  void followJointTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
    ROS_INFO("Executing follow joint trajectory action");
    // Wait until the action is complete, sending status/feedback along the
    // way.

    /////////////////////////////////
    // Remap
    /////////////////////////////////
    
    // We must remap the joint from the ROS message to our J1-J6 ordering for our robot
    auto num_joints = goal->trajectory.points[0].positions.size();
    std::vector<int> ros_to_hebi(num_joints);
    auto& src_names = goal->trajectory.joint_names;
    auto& dst_names = joint_state_message_.name;
    for (int i = 0; i < src_names.size(); ++i)
    {
      auto location = std::find(dst_names.begin(), dst_names.end(), src_names[i]);
      if (location == dst_names.end())
      {
        action_server_.setAborted();
        return;
      }
      ros_to_hebi[i] = location - dst_names.begin();
    }
    
    /////////////////////////////////
    // Replan trajectory
    /////////////////////////////////

    // Get the positions, velocities, and accelerations to pass into the HEBI
    // trajectory generator. One important note -- we drop the first waypoint here
    // b/c the hebi Arm API adds a waypoint at the current position already.
    auto num_pts = goal->trajectory.points.size();
    Eigen::MatrixXd positions(num_joints, num_pts - 1);
    Eigen::MatrixXd velocities(num_joints, num_pts - 1);
    Eigen::MatrixXd accelerations(num_joints, num_pts - 1);
    Eigen::VectorXd times(num_pts - 1);
    for (int wp = 1; wp < num_pts; ++wp) // Drop the first waypoint! See above comment!
    {
      auto& waypoint = goal->trajectory.points[wp];
      times(wp - 1) = waypoint.time_from_start.toSec();
      for (int j = 0; j < num_joints; ++j)
      {
        auto dest_joint = ros_to_hebi[j];
        positions(dest_joint, wp - 1) = waypoint.positions[j];
        velocities(dest_joint, wp - 1) = waypoint.velocities[j];
        accelerations(dest_joint, wp - 1) = waypoint.accelerations[j];
      }
    }

    arm::Goal arm_goal(times, positions, velocities, accelerations);
    arm_.setGoal(arm_goal);

    /////////////////////////////////
    // Execute trajectory
    /////////////////////////////////

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);
    control_msgs::FollowJointTrajectoryFeedback feedback;

    ROS_INFO("HEBI MoveIt Arm Node executing trajectory");

    while (true) {
      if (action_server_.isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Follow joint trajectory action was preempted");
        // Note -- the `followJointTrajectory` function will not be called until the
        // action server has been preempted here:
        action_server_.setPreempted();
        return;
      }

      if (!action_server_.isActive() || !::ros::ok()) {
        ROS_INFO("Follow joint trajectory was cancelled");
        return;
      }

      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      // TODO: we could fill in the action server feedback with desired, actual, and
      // error values if necessary. For now, we rely on the joint_states message to
      // accomplish this.
      // action_server_.publishFeedback(feedback);
 
      if (arm_.atGoal()) {
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }
    
    // TODO: We could fill in the action server's result with an error code and error string if
    // desired.  Also, we could/should check the path and goal tolerance here.

    // publish when the arm is done with a motion
    ROS_INFO("Completed follow joint trajectory action");
    action_server_.setSucceeded();
  }

  // The "heartbeat" of the program -- sends out messages and updates feedback from
  // and commands to robot
  bool update(::ros::Time t) {
    static int seq = 0;

    // Update arm, and send new commands
    bool res = arm_.update(t.toSec());
    if (res)
      res = arm_.send();

    // Update position and publish

    // Note -- we don't need to reorder here, b/c we are using the HEBI ordering
    // defined when we filled in the message in the MoveItArmNode constructor
    auto& gf = arm_.lastFeedback();
    auto pos = gf.getPosition();
    auto vel = gf.getVelocity();
    auto eff = gf.getEffort();
    for (int i = 0; i < gf.size(); ++i)
    {
      joint_state_message_.position[i] = pos[i];
      joint_state_message_.velocity[i] = vel[i];
      joint_state_message_.effort[i] = eff[i];
    }
    joint_state_message_.header.seq = ++seq;
    joint_state_message_.header.stamp = t;
    joint_state_publisher_.publish(joint_state_message_);

    return res;
  }

private:
  arm::Arm& arm_;

  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;

  ::ros::Publisher joint_state_publisher_;

  sensor_msgs::JointState joint_state_message_;
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_moveit_node");
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

  std::vector<std::string> moveit_joints;
  if (node.hasParam("moveit_joints") &&
      node.getParam("moveit_joints", moveit_joints) &&
      moveit_joints.size() == names.size()) {
    ROS_INFO("Found and successfully read 'moveit_joints' parameter");
  } else {
    ROS_ERROR("Could not find/read required 'moveit_joints' parameter or parameter did not match length of 'names'; aborting!");
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

  // Create arm
  hebi::arm::Arm::Params params;
  params.families_ = families;
  params.names_ = names;
  params.hrdf_file_ = ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file;

  auto arm = hebi::arm::Arm::create(ros::Time::now().toSec(), params);
  for (int num_tries = 0; num_tries < 3; num_tries++) {
    arm = hebi::arm::Arm::create(ros::Time::now().toSec(), params);
    if (arm) {
      break;
    }
    ROS_WARN("Could not initialize arm, trying again...");
    ros::Duration(1.0).sleep();
  }

  if (!arm) {
    ROS_ERROR("Could not initialize arm! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }
  
  // Load the appropriate gains file
  if (!arm->loadGains(ros::package::getPath(gains_package) + std::string("/") + gains_file)) {
    ROS_ERROR("Could not load gains file and/or set arm gains. Attempting to continue.");
  }

  // Get the home position, defaulting to (nearly) zero
  Eigen::VectorXd home_position(arm->size());
  if (home_position_vector.empty()) {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = 0.01; // Avoid common singularities by being slightly off from zero
    }
  } else if (home_position_vector.size() != arm->size()) {
    ROS_ERROR("'home_position' parameter not the same length as HRDF file's number of DoF! Aborting!");
    return -1;
  } else {
    for (size_t i = 0; i < home_position.size(); ++i) {
      home_position[i] = home_position_vector[i];
    }
  }

  /////////////////// Initialize ROS interface ///////////////////
  
  hebi::ros::MoveItArmNode arm_node(*arm, node, moveit_joints);

  /////////////////// Main Loop ///////////////////

  // We update with a current timestamp so the "setGoal" function
  // is planning from the correct time for a smooth start
  arm_node.update(ros::Time::now());
  arm->setGoal({home_position});

  while (ros::ok()) {

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm_node.update(ros::Time::now()))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
