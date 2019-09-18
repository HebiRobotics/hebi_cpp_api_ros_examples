#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>

#include "src/util/mobile_io.hpp"

int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "controller");
  ros::NodeHandle node;

  // Create mobile IO connection
  auto io = hebi::MobileIO::create("HEBI", "Mobile IO");
  using ButtonState = hebi::MobileIODiff::ButtonState;

  /////////////////// Initialize ROS interface ///////////////////
  ros::Publisher cmd_vel_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  geometry_msgs::Twist cmd_vel_msg;

  /////////////////// Main Loop ///////////////////

  // TODO: set IO LED green when found, white when exiting?

  // Main command loop
  auto last_state = io->getState();

  uint8_t send_count = 0;
  uint8_t send_period = 10;

  while(ros::ok()) {
    // Get next IO feedback state
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)

    auto state = io->getState();
    hebi::MobileIODiff diff(last_state, state);
    last_state = state;

    cmd_vel_msg.linear.y = -state.axes_[6];
    cmd_vel_msg.linear.x = state.axes_[7];
    cmd_vel_msg.angular.z = -state.axes_[0] * 2.0;
    
    if (send_count == 0)
      cmd_vel_pub.publish(cmd_vel_msg);

    // Quit
    if (diff.get(8) == ButtonState::ToOn) {
      break;
    }

    ++send_count;
    send_count = send_count % send_period;

    // Call any pending callbacks (note -- there are none right now)
    ros::spinOnce();
  }

  return 0;
}
