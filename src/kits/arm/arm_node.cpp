#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>

#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>

#include <hebi_cpp_api_examples/TargetWaypoints.h>
#include <hebi_cpp_api_examples/ArmMotionAction.h>


#include "hebi_cpp_api/group_command.hpp"

#include "robot/arm.hpp"
#include "hebi_cpp_api/robot_model.hpp"

namespace hebi {
namespace ros {

class ArmNode {
public:
  ArmNode(arm::Arm& arm, const Eigen::VectorXd& home_position) : arm_(arm), home_position_(home_position) { }

  // Callback for trajectories with joint angle waypoints
  void updateJointWaypoints(trajectory_msgs::JointTrajectory joint_trajectory) {
    auto num_joints = arm_.size();
    auto num_waypoints = joint_trajectory.points.size();
    Eigen::MatrixXd pos(num_joints, num_waypoints);
    Eigen::MatrixXd vel(num_joints, num_waypoints);
    Eigen::MatrixXd accel(num_joints, num_waypoints);
    Eigen::VectorXd times(num_waypoints);
    for (size_t waypoint = 0; waypoint < num_waypoints; ++waypoint) {
      auto& cmd_waypoint = joint_trajectory.points[waypoint];

      if (cmd_waypoint.positions.size() != num_joints ||
          cmd_waypoint.velocities.size() != num_joints ||
          cmd_waypoint.accelerations.size() != num_joints) {
        ROS_ERROR_STREAM("Position, velocity, and acceleration sizes not correct for waypoint index " << waypoint);
        return;
      }

      if (cmd_waypoint.effort.size() != 0) {
        ROS_WARN_STREAM("Effort commands in trajectories not supported; ignoring");
      }

      for (size_t joint = 0; joint < num_joints; ++joint) {
        pos(joint, waypoint) = joint_trajectory.points[waypoint].positions[joint];
        vel(joint, waypoint) = joint_trajectory.points[waypoint].velocities[joint];
        accel(joint, waypoint) = joint_trajectory.points[waypoint].accelerations[joint];
      }

      times(waypoint) = cmd_waypoint.time_from_start.toSec();
    }
    updateJointWaypoints(pos, vel, accel, times);
  }

  void updateCartesianWaypoints(hebi_cpp_api_examples::TargetWaypoints target_waypoints) {
    if (action_server_->isActive())
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
    updateCartesianWaypoints(xyz_waypoints);
  }

  // Set the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void setTargetCallback(geometry_msgs::Point data) {
    if (action_server_->isActive())
      action_server_->setAborted();

    // Fill in an Eigen::Matrix3xd with the xyz goal
    Eigen::Matrix3Xd xyz_waypoints(3, 1);
    xyz_waypoints(0, 0) = data.x;
    xyz_waypoints(1, 0) = data.y;
    xyz_waypoints(2, 0) = data.z;

    // Replan
    updateCartesianWaypoints(xyz_waypoints);
  }

  // "Jog" the target end effector location in (x,y,z) space, replanning
  // smoothly to the new location
  void offsetTargetCallback(geometry_msgs::Point data) {
    if (action_server_->isActive())
      action_server_->setAborted();

    // Only update if target changes!
    if (data.x == 0 && data.y == 0 && data.z == 0)
      return;

    // Initialize target from feedback as necessary
    if (!isTargetInitialized()) {
      auto pos = arm_.FK3Dof(arm_.lastFeedback().getPositionCommand());
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
    updateCartesianWaypoints(xyz_waypoints);
  }

  void setCompliantMode(std_msgs::Bool msg) {
    if (msg.data) {
      // Go into a passive mode so the system can be moved by hand
      ROS_INFO("Pausing active command (entering grav comp mode)");
      arm_.cancelGoal();
    } else {
      ROS_INFO("Resuming active command"); 
      auto t = ::ros::Time::now().toSec();
      auto last_position = arm_.lastFeedback().getPosition();
      arm_.setGoal({last_position});
      target_xyz_ = arm_.FK3Dof(last_position);
    }
  }

  void setColor(const Color& color) {
    auto& command = arm_.pendingCommand();
    for (int i = 0; i < command.size(); ++i) {
      command[i].led().set(color);
    }
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
        xyz_positions.col(i) = home_position_;
        tip_directions(0, i) = 1;
        tip_directions(1, i) = 0;
        tip_directions(2, i) = 0;
      }
      else if (goal->x[i] == 101 && goal->y[i] == 101 && goal->z[i] == 101) {
        xyz_positions.col(i) = home_position_;
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

    updateCartesianWaypoints(xyz_positions, &tip_directions);

    // Set LEDs to a particular color, or clear them.
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    setColor(color);

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);

    hebi_cpp_api_examples::ArmMotionFeedback feedback;

    while (true) {
      if (action_server_->isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Arm motion action was preempted");
        setColor({0,0,0,0});
        // Note -- the `startArmMotion` function will not be called until the
        // action server has been preempted here:
        action_server_->setPreempted();
        return;
      }

      if (!action_server_->isActive() || !::ros::ok()) {
        ROS_INFO("Arm motion was cancelled");
        setColor({0,0,0,0});
        return;
      }

      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      feedback.percent_complete = arm_.goalProgress() * 100.0;
      action_server_->publishFeedback(feedback);
 
      if (arm_.atGoal()) {
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }
    
    setColor({0,0,0,0});

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
  // unitialized state, and will be set from feedback during first
  // command)
  Eigen::Vector3d target_xyz_{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  Eigen::VectorXd home_position_;

  actionlib::SimpleActionServer<hebi_cpp_api_examples::ArmMotionAction>* action_server_ {nullptr};

  // Helper function that indicates whether or not was have received a
  // command yet.
  bool isTargetInitialized() {
    return !std::isnan(target_xyz_.x()) ||
           !std::isnan(target_xyz_.y()) ||
           !std::isnan(target_xyz_.z());
  }

  // Each row is a separate joint; each column is a separate waypoint.
  void updateJointWaypoints(const Eigen::MatrixXd& angles, const Eigen::MatrixXd& velocities, const Eigen::MatrixXd& accelerations, const Eigen::VectorXd& times) {
    // Data sanity check:
    if (angles.rows() != velocities.rows()       || // Number of joints
        angles.rows() != accelerations.rows()    ||
        angles.rows() != arm_.size() ||
        angles.cols() != velocities.cols()       || // Number of waypoints
        angles.cols() != accelerations.cols()    ||
        angles.cols() != times.size()            ||
        angles.cols() == 0) {
      ROS_ERROR("Angles, velocities, accelerations, or times were not the correct size");
      return;
    }

    // Update stored target position, based on final joint angles.
    target_xyz_ = arm_.FK3Dof(angles.rightCols<1>());

    // Replan:
    arm::Goal arm_goal(times, angles, velocities, accelerations);
    arm_.setGoal(arm_goal);
  }

  // Helper function to condense functionality between various message/action callbacks above
  // Replan a smooth joint trajectory from the current location through a
  // series of cartesian waypoints.
  // xyz positions should be a 3xn vector of target positions
  void updateCartesianWaypoints(const Eigen::Matrix3Xd& xyz_positions,
      const Eigen::Matrix3Xd* end_tip_directions = nullptr) {
    // Data sanity check:
    if (end_tip_directions && end_tip_directions->cols() != xyz_positions.cols())
      return;

    // Update stored target position:
    target_xyz_ = xyz_positions.col(xyz_positions.cols() - 1);

    // These are the joint angles that will be added
    auto num_waypoints = xyz_positions.cols();
    Eigen::MatrixXd positions(arm_.size(), num_waypoints);

    // Plan to each subsequent point from the last position
    // (We use the last position command for smoother motion)
    Eigen::VectorXd last_position = arm_.lastFeedback().getPositionCommand();

    // For each waypoint, find the joint angles to move to it, starting from the last
    // waypoint, and save into the position vector.
    if (end_tip_directions) {
      // If we are given tip directions, add these too...
      for (size_t i = 0; i < num_waypoints; ++i) {
        last_position = arm_.solveIK5Dof(last_position, xyz_positions.col(i), end_tip_directions->col(i));
        positions.col(i) = last_position; 
      }
    } else {
      for (size_t i = 0; i < num_waypoints; ++i) {
        last_position = arm_.solveIK3Dof(last_position, xyz_positions.col(i));
        positions.col(i) = last_position; 
      }
    }

    // Replan:
    arm_.setGoal({positions});
  }
 
};

} // namespace ros
} // namespace hebi

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

  // Create arm
  hebi::arm::Arm::Params params;
  params.families_ = families;
  params.names_ = names;
  params.hrdf_file_ = ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file;

  auto arm = hebi::arm::Arm::create(ros::Time::now().toSec(), params);
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
   
  hebi::ros::ArmNode arm_node(*arm, home_position);

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
  ros::Subscriber cartesian_waypoint_subscriber =
    node.subscribe<hebi_cpp_api_examples::TargetWaypoints>("cartesian_waypoints", 50, &hebi::ros::ArmNode::updateCartesianWaypoints, &arm_node);

  // Subscribe to lists of joint angle waypoints
  ros::Subscriber joint_waypoint_subscriber =
    node.subscribe<trajectory_msgs::JointTrajectory>("joint_waypoints", 50, &hebi::ros::ArmNode::updateJointWaypoints, &arm_node);

  // Subscribe to "compliant mode" toggle, so the robot is placed in/out of
  // a "grav comp only" mode.
  ros::Subscriber compliant_mode_subscriber =
    node.subscribe<std_msgs::Bool>("compliant_mode", 50, &hebi::ros::ArmNode::setCompliantMode, &arm_node);

  /////////////////// Main Loop ///////////////////

  // We update with a current timestamp so the "setGoal" function
  // is planning from the correct time for a smooth start
  arm->update(ros::Time::now().toSec());
  arm->setGoal({home_position});

  while (ros::ok()) {
    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update(ros::Time::now().toSec()))
      ROS_WARN("Error Getting Feedback -- Check Connection");
    else if (!arm->send())
      ROS_WARN("Error Sending Commands -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
