#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Inertia.h>
#include <std_srvs/SetBool.h>

#include <hebi_cpp_api_examples/TargetWaypoints.h>
#include <hebi_cpp_api_examples/ArmMotionAction.h>
#include <hebi_cpp_api_examples/SetIKSeed.h>

#include "hebi_cpp_api/group_command.hpp"

#include "hebi_cpp_api/robot_model.hpp"

#include "hebi_cpp_api/arm/arm.hpp"
#include "hebi_cpp_api/util/mobile_io.hpp"

namespace arm = hebi::experimental::arm;
namespace experimental = hebi::experimental;

namespace hebi {
namespace ros {

class ArmNode {
public:
  ArmNode(::ros::NodeHandle* nh, arm::Arm& arm, const Eigen::VectorXd& home_position, std::vector<std::string> link_names) : nh_(*nh),
       arm_(arm),
       home_position_(home_position),
       compliant_mode_service_(nh->advertiseService("compliance_mode", &ArmNode::setCompliantMode, this)),
       ik_seed_service_(nh->advertiseService("set_ik_seed", &hebi::ros::ArmNode::handleIKSeedService, this)),
       arm_state_pub_(nh->advertise<sensor_msgs::JointState>("joint_states", 50)),
       center_of_mass_publisher_(nh->advertise<geometry_msgs::Inertia>("inertia", 100)) {

    //TODO: Figure out a way to get link names from the arm, so it doesn't need to be input separately
    state_msg_.name = link_names;
  }

  bool setIKSeed(const std::vector<double>& ik_seed) {
    if(ik_seed.size() != home_position_.size()) {
      use_ik_seed_ = false;
    } else {
      use_ik_seed_ = true;
      ik_seed_.resize(ik_seed.size());
      for (size_t i = 0; i < ik_seed.size(); ++i) {
        ik_seed_[i] = ik_seed[i];
      }
    }
    return use_ik_seed_;
  }

  bool handleIKSeedService(hebi_cpp_api_examples::SetIKSeed::Request& req, hebi_cpp_api_examples::SetIKSeed::Response& res) {
    return setIKSeed(req.seed);
  }

  bool setCompliantMode(std_srvs::SetBool::Request &req,
		        std_srvs::SetBool::Response &res) {
    if (req.data) {
      // Go into a passive mode so the system can be moved by hand
      res.message = "Pausing active command (entering grav comp mode)";
      arm_.cancelGoal();
      res.success = true;
    } else {
      res.message = "Resuming active command";
      auto t = ::ros::Time::now().toSec();
      auto last_position = arm_.lastFeedback().getPosition();
      arm_.setGoal(arm::Goal::createFromPosition(last_position));
      target_xyz_ = arm_.FK(last_position);
      res.success = true;
    }
    return true;
  }

  void setColor(const Color& color) {
    auto& command = arm_.pendingCommand();
    for (int i = 0; i < command.size(); ++i) {
      command[i].led().set(color);
    }
  }

  void publishState() {
    auto& fdbk = arm_.lastFeedback();

    // how is there not a better way to do this?

    // Need to copy data from VectorXd to vector<double> in ros msg
    auto pos = fdbk.getPosition();
    auto vel = fdbk.getVelocity();
    auto eff = fdbk.getEffort();

    state_msg_.position.resize(pos.size());
    state_msg_.velocity.resize(vel.size());
    state_msg_.effort.resize(eff.size());
    state_msg_.header.stamp = ::ros::Time::now();

    VectorXd::Map(&state_msg_.position[0], pos.size()) = pos;
    VectorXd::Map(&state_msg_.velocity[0], vel.size()) = vel;
    VectorXd::Map(&state_msg_.effort[0], eff.size()) = eff;

    arm_state_pub_.publish(state_msg_);

    // compute arm CoM
    auto& model = arm_.robotModel();
    Eigen::VectorXd masses;
    robot_model::Matrix4dVector frames;
    model.getMasses(masses);
    model.getFK(robot_model::FrameType::CenterOfMass, pos, frames);

    center_of_mass_message_.m = 0.0;
    Eigen::Vector3d weighted_sum_com = Eigen::Vector3d::Zero();
    for(int i = 0; i < model.getFrameCount(robot_model::FrameType::CenterOfMass); ++i) {
      center_of_mass_message_.m += masses(i);
      frames[i] *= masses(i);
      weighted_sum_com(0) += frames[i](0, 3);
      weighted_sum_com(1) += frames[i](1, 3);
      weighted_sum_com(2) += frames[i](2, 3);
    }
    weighted_sum_com /= center_of_mass_message_.m;

    center_of_mass_message_.com.x = weighted_sum_com(0);
    center_of_mass_message_.com.y = weighted_sum_com(1);
    center_of_mass_message_.com.z = weighted_sum_com(2);

    center_of_mass_publisher_.publish(center_of_mass_message_);
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

  Eigen::VectorXd ik_seed_{0};
  bool use_ik_seed_{false};

  ::ros::NodeHandle nh_;

  sensor_msgs::JointState state_msg_;
  geometry_msgs::Inertia center_of_mass_message_;

  ::ros::Publisher arm_state_pub_;
  ::ros::Publisher center_of_mass_publisher_;

  ::ros::ServiceServer compliant_mode_service_;
  ::ros::ServiceServer ik_seed_service_;
};

} // namespace ros
} // namespace hebi

template <typename T>
bool loadParam(ros::NodeHandle node, std::string varname, T& var) {
  if (node.hasParam(varname) && node.getParam(varname, var)) {
    ROS_INFO_STREAM("Found and successfully read '" << varname << "' parameter");
    return true;
  }

  ROS_ERROR_STREAM("Could not find/read required '" << varname << "' parameter!");
  return false;
}


Eigen::Matrix3d makeRotationMatrix (hebi::Quaternionf phone_orientation) {
  Eigen::Quaterniond q;
  q.w() = phone_orientation.getW();
  q.x() = phone_orientation.getX();
  q.y() = phone_orientation.getY();
  q.z() = phone_orientation.getZ();
  return q.toRotationMatrix();
}


int main(int argc, char ** argv) {

  // Initialize ROS node
  ros::init(argc, argv, "arm_ar_node");
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
  // Read the package + path for the gains file
  std::string gains_package;
  std::string gains_file;
  // Read the package + path for the hrdf file
  std::string hrdf_package;
  std::string hrdf_file;

  bool success = true;
  success = success && loadParam(node, "names", names);
  success = success && loadParam(node, "gains_package", gains_package);
  success = success && loadParam(node, "gains_file", gains_file);
  success = success && loadParam(node, "hrdf_package", hrdf_package);
  success = success && loadParam(node, "hrdf_file", hrdf_file);

  if(!success) {
    ROS_ERROR("Could not find one or more required parameters; aborting!");
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
  arm::Arm::Params params;
  params.families_ = families;
  params.names_ = names;
  params.get_current_time_s_ = []() {
    static double start_time = ros::Time::now().toSec();
    return ros::Time::now().toSec() - start_time;
  };

  params.hrdf_file_ = ros::package::getPath(hrdf_package) + std::string("/") + hrdf_file;

  auto arm = arm::Arm::create(params);
  for (int num_tries = 0; num_tries < 3; num_tries++) {
    arm = arm::Arm::create(params);
    if (arm) {
      break;
    }
    ROS_WARN("Could not initialize arm, trying again...");
    ros::Duration(1.0).sleep();
  }

  if (!arm) {
    ROS_ERROR_STREAM("Failed to find the following modules in family: " << families.at(0));
    for(auto it = names.begin(); it != names.end(); ++it) {
        ROS_ERROR_STREAM("> " << *it);
    }
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

  // Make a list of family/actuator formatted names for the JointState publisher
  std::vector<std::string> full_names;
  for (size_t idx=0; idx<names.size(); ++idx) {
    full_names.push_back(families.at(0) + "/" + names.at(idx));
  }

  /////////////////// Initialize ROS interface ///////////////////

  hebi::ros::ArmNode arm_node(&node, *arm, home_position, full_names);

  if (node.hasParam("ik_seed")) {
    std::vector<double> ik_seed;
    node.getParam("ik_seed", ik_seed);
    arm_node.setIKSeed(ik_seed);
  } else {
    ROS_WARN("Param ik_seed not set, arm may exhibit erratic behavior");
  }

  /////////////////// mobileIO  ///////////////////

  std::unique_ptr<experimental::MobileIO> mobile = experimental::MobileIO::create(params.families_[0], "mobileIO");
  while(!mobile) {
    ROS_WARN("Can't find mobileIO, trying again...");
    ros::Duration(1.0).sleep();
    mobile = experimental::MobileIO::create(params.families_[0], "mobileIO");
  }
  mobile->clearText();

  // Setup instructions for display
  std::string instructions;
  instructions = ("B1 - Home Position\nB3 - AR Control Mode\n"
                  "B6 - Grav Comp Mode\nB8 - Quit\n");

  // Display instructions on screen
  mobile -> sendText(instructions); 

  // Setup state variable for mobile device
  auto last_mobile_state = mobile->getState();

  // Different modes it can be in (when in none, then automatic grav_comp)
  bool softstart = true;
  bool ar_mode = false;
  
  auto t = ros::Time::now();

  // Command the softstart to home position
  arm -> update();
  arm -> setGoal(arm::Goal::createFromPosition(4, home_position)); // take 4 seconds
  arm -> send();

  // Get the cartesian position and rotation matrix @ home position
  Eigen::Vector3d xyz_home; 
  Eigen::Matrix3d rot_home;
  arm -> FK(home_position, xyz_home, rot_home); 

  // Set up states for the mobile device
  Eigen::Vector3d xyz_phone_init;
  Eigen::Matrix3d rot_phone_init;
  Eigen::Matrix3d rot_init_target;//(3,3);
  rot_init_target << 0,-1,0, 1,0,0, 0,0,1;


  /////////////////// Main Loop ///////////////////

  // We update with a current timestamp so the "setGoal" function
  // is planning from the correct time for a smooth start
 
  // Target variables
  Eigen::VectorXd target_joints(arm -> robotModel().getDoFCount());

  auto prev_t = t;

  while (ros::ok()) {
    t = ros::Time::now();

    // Update feedback, and command the arm to move along its planned path
    // (this also acts as a loop-rate limiter so no 'sleep' is needed)
    if (!arm->update())
      ROS_WARN("Error Getting Feedback -- Check Connection");
    else if (!arm->send())
      ROS_WARN("Error Sending Commands -- Check Connection");

    arm_node.publishState();

       if (softstart) {
      Eigen::VectorXd home_position(arm -> robotModel().getDoFCount());
      // End softstart when arm reaches its homePosition
      if (arm -> atGoal()){
        mobile -> sendText("Softstart Complete!");
        softstart = false;
        continue;
      }
      arm -> send();

      // Stay in softstart, don't do any other behavior
      continue;
    }

    // Get latest mobile_state
    auto mobile_state = mobile->getState();
    experimental::MobileIODiff diff(last_mobile_state, mobile_state);

    // Get latest gripper_state
    // gripper -> getState();

    // Button B1 - Return to home position
    if (diff.get(1) == experimental::MobileIODiff::ButtonState::ToOn) {
        ar_mode = false;
        arm -> setGoal(arm::Goal::createFromPosition(4, home_position));
    }

    // Button B3 - Start AR Control
    if (diff.get(3) == experimental::MobileIODiff::ButtonState::ToOn) {
      xyz_phone_init << mobile -> getLastFeedback().mobile().arPosition().get().getX(),
                        mobile -> getLastFeedback().mobile().arPosition().get().getY(),
                        mobile -> getLastFeedback().mobile().arPosition().get().getZ();
      std::cout << xyz_phone_init << std::endl;
      rot_phone_init = makeRotationMatrix(mobile -> getLastFeedback().mobile().arOrientation().get());
      ar_mode = true;
    }

    // Button B6 - Grav Comp Mode
    if (diff.get(6) == experimental::MobileIODiff::ButtonState::ToOn) {
      arm -> cancelGoal();
      ar_mode = false;
    }

    // Button B8 - End Demo
    if (diff.get(8) == experimental::MobileIODiff::ButtonState::ToOn) {
      // Clear MobileIO text
      mobile -> clearText();
      return 1;
    }

    if (ar_mode) {
      // Get the latest mobile position and orientation
      Eigen::Vector3d phone_pos;
      phone_pos << mobile -> getLastFeedback().mobile().arPosition().get().getX(),
                   mobile -> getLastFeedback().mobile().arPosition().get().getY(),
                   mobile -> getLastFeedback().mobile().arPosition().get().getZ();
      auto phone_ori = mobile -> getLastFeedback().mobile().arOrientation().get();

      // Calculate rotation matrix from orientation quaternion
      auto rot_phone_target = makeRotationMatrix(phone_ori);

      // Calculate new targets
      Eigen::Vector3d xyz_scale;
      xyz_scale << 1, 1, 2;
      Eigen::Matrix3d rot_target = rot_phone_init.transpose() * rot_phone_target * rot_init_target;
      Eigen::Vector3d xyz_target = xyz_home + (0.75 * xyz_scale.array() *
                                (rot_phone_init.transpose() * (phone_pos - xyz_phone_init)).array()).matrix();

      // Calculate new arm joint angle targets
      target_joints = arm -> solveIK(arm -> lastFeedback().getPosition(),
                                              xyz_target,
                                              rot_target);

      // Create and send new goal to the arm
      arm -> setGoal(arm::Goal::createFromPosition(target_joints));

      // double open_val = (mobile_state.getAxis(3) * 2)-1; // makes this range between -2 and 1
      // gripper -> update(open_val);
    }

    // Update mobile device to the new last_state
    last_mobile_state = mobile_state;

    // Send latest commands to the arm
    arm->send();

    // If a simulator reset has occured, go back to the home position.
    if (t < prev_t) {
      std::cout << "Returning to home pose after simulation reset" << std::endl;
      arm->setGoal(arm::Goal::createFromPosition(home_position));
    }
    prev_t = t;

    // Call any pending callbacks (note -- this may update our planned motion)
    ros::spinOnce();
  }

  return 0;
}
