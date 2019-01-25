#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <hebi_cpp_api_ros_examples/BaseMotionAction.h>

#include "actionlib/server/simple_action_server.h"

#include "lookup.hpp"
#include "group.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "trajectory.hpp"

#include "robot/omni_base.hpp"

#include <ros/console.h>
#include <ros/package.h>

namespace hebi {
namespace ros {

// TODO: fix includes here and in Rosie_arm_node file!

class BaseNode {
public:
  BaseNode(OmniBase& base) : base_(base) {
    Color c;
    base_.resetStart(c);
  }

  void startBaseMotion(const hebi_cpp_api_ros_examples::BaseMotionGoalConstPtr& goal) {

    // Note: this is implemented right now as translation, _THEN_ rotation...
    // we can update this later.

    // We pass in the trajectory points in (x, y, theta)... 
    size_t num_waypoints = 1;
    Eigen::MatrixXd waypoints(3, num_waypoints);

    ROS_INFO("Executing base motion action");
    hebi_cpp_api_ros_examples::BaseMotionFeedback feedback;

    ////////////////
    // Translation
    ////////////////
    Color color;
    if (goal->set_color)
      color = Color(goal->r, goal->g, goal->b, 255);
    base_.resetStart(color);

    waypoints(0, 0) = goal->x;
    waypoints(1, 0) = goal->y;
    waypoints(2, 0) = goal->theta;
    base_.getTrajectory().replan(
      ::ros::Time::now().toSec(),
      waypoints);

    // Wait until the action is complete, sending status/feedback along the
    // way.
    ::ros::Rate r(10);
    bool success = true;
    while (true) {
      if (action_server_->isPreemptRequested() || !::ros::ok()) {
        ROS_INFO("Preempted base motion");
        action_server_->setPreempted();
        success = false;
        break;
      }
      auto t = ::ros::Time::now().toSec();

      // Publish progress:
      auto& base_traj = base_.getTrajectory();
      feedback.percent_complete = base_.trajectoryPercentComplete(t) / 2.0;
      action_server_->publishFeedback(feedback);
 
      if (base_.isTrajectoryComplete(t)) {
        ROS_INFO("TRANSLATION COMPLETE");
        break;
      }

      // Limit feedback rate
      r.sleep(); 
    }

    base_.clearColor();

    // publish when the base is done with a motion
    ROS_INFO("Completed base motion action");
    hebi_cpp_api_ros_examples::BaseMotionResult result;
    result.success = success;
    action_server_->setSucceeded(result); // TODO: set failed?
  }

  void setActionServer(actionlib::SimpleActionServer<hebi_cpp_api_ros_examples::BaseMotionAction>* action_server) {
    action_server_ = action_server;
  }

private:
  OmniBase& base_; 

  actionlib::SimpleActionServer<hebi_cpp_api_ros_examples::BaseMotionAction>* action_server_ {nullptr};
};

} // namespace ros
} // namespace hebi

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "Rosie_base_node");
  ros::NodeHandle node;

  /////////////////// Initialize base ///////////////////

  // Create base and plan initial trajectory
  std::string error_out;
  auto base = hebi::OmniBase::create(
    {"Rosie"},                         // Family
    {"Wheel1", "Wheel2", "Wheel3"}, // Names
    ::ros::package::getPath("hebi_cpp_api_ros_examples") + "/gains/omni_base_gains.xml", // Gains file
    ros::Time::now().toSec(),
    error_out);         // Starting time (for trajectory)
  if (!base) {
    ROS_ERROR_STREAM(error_out);
    ROS_ERROR("Could not initialize base! Check for modules on the network, and ensure good connection (e.g., check packet loss plot in Scope). Shutting down...");
    return -1;
  }

  /////////////////// Initialize ROS interface ///////////////////
   
  hebi::ros::BaseNode base_node(*base);

  // Action server for base motions
  actionlib::SimpleActionServer<hebi_cpp_api_ros_examples::BaseMotionAction> base_motion_action(
    node, "omni_base/base_motion",
    boost::bind(&hebi::ros::BaseNode::startBaseMotion, &base_node, _1), false);

  base_node.setActionServer(&base_motion_action);

  base_motion_action.start();

  /////////////////// Main Loop ///////////////////

  double t_now;

  // Main command loop
  while (ros::ok()) {

    auto t = ros::Time::now().toSec();

    // Update feedback, and command the base to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!base->update(t))
      ROS_WARN("Error Getting Feedback -- Check Connection");

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
