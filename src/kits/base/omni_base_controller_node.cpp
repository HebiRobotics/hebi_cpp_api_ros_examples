//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
//#include <ros/console.h>

#include <geometry_msgs/msg/twist.hpp>

#include "src/util/mobile_io.hpp"
#include "src/util/param_loaders.hpp"

int main(int argc, char ** argv) {

  // Initialize ROS node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("controller");

  std::string family = loadScalarParamWarn(node, "family", "OmniDrive");

  //node.declare_parameter("family");

  //std::string family = "OmniDrive";
  //auto val = node->get_parameter("family");
  //if (val.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
  //  family = val.as_string();
  //} else {
  //  RCLCPP_WARN(node->get_logger(), "Could not find/read 'family' string parameter; defaulting to 'OmniDrive'");
  //}

  // Create mobile IO connection
  auto io = hebi::MobileIO::create(family, "mobileIO");
  using ButtonState = hebi::MobileIODiff::ButtonState;

  /////////////////// Initialize ROS interface ///////////////////
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
  geometry_msgs::msg::Twist cmd_vel_msg;

  /////////////////// Main Loop ///////////////////

  // TODO: set IO LED green when found, white when exiting?

  // Main command loop
  auto last_state = io->getState();

  uint8_t send_count = 0;
  uint8_t send_period = 10;

  while(rclcpp::ok()) {
    // Get next IO feedback state
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)

    auto state = io->getState();
    hebi::MobileIODiff diff(last_state, state);
    last_state = state;

    auto dy = -state.axes_[6];
    auto dx = state.axes_[7];
    auto dtheta = -state.axes_[0];

    cmd_vel_msg.linear.x = pow(dx, 3);
    cmd_vel_msg.linear.y = pow(dy, 3);
    cmd_vel_msg.angular.z = pow(dtheta, 3) * 2.0;
    
    if (send_count == 0)
      cmd_vel_pub->publish(cmd_vel_msg);

    // Quit
    if (diff.get(8) == ButtonState::ToOn) {
      break;
    }

    ++send_count;
    send_count = send_count % send_period;

    // Call any pending callbacks (note -- there are none right now)
    rclcpp::spin_some(node);
  }

  return 0;
}
