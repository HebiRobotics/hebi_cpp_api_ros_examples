# Overview

These examples demonstrate use of the C++ API directly from ROS nodes. These wrap up this functionality and low-level module communication, and expose a higher-level "system" interface on ROS topics, actions, and services.

**Important: to run any of these commands, you will need to run `source devel/setup.sh` from your catkin workspace first**

# Arm (`arm_node`)

## Requirements:

Set the `families` and `names` parameters to lists of strings, ordered from proximal to distal, of your robot. Have modules on the network matching these names.

Note: if all families are the same, this parameter can be a length one vector.

You may also need to modify the parameters for the location of the gains file, HRDF file, and home position of the system.

Default values for all parameters are given in the `launch/arm_node*.launch` files.

## To run:

```
roslaunch hebi_cpp_api_ros_examples arm_node<configuration>.launch
```

## To command:


Note: you can use `rostopic pub` on the command line for the following `publish` lines; press tab to see the available channels, message types, and to create a basic filled out YAML message that you can edit.  For more advanced editing, copy the message into a text editor and paste back the full message.

### `offset_target`

- Publish `geometry_msgs::Point` messages on the `offset_target` channel to jog the end effector in a certain cartesian direction relative to the base. (The initial "offset" value is taken from the actuator's current position.)

### `set_target`

- Publish `geometry_msgs::Point messages on the `set_target` channel to set the end effector cartesian position relative to the base.

### `cartesian_waypoints`

- Publish `hebi_cpp_api_ros_examples::TargetWaypoint` messages on the `cartesian_waypoints` channel to command a series of cartesian waypoints for the arm to pass through.

### `joint_waypoints`

- Publish trajectory_msgs::JointTrajectory messages on the `joint_waypoints` channel to command a series of joint-space waypoints for the arm to pass through.

Note: for the JointTrajectory messages, you can ignore the header and the "names" fields, as well as the "efforts".  You must fill in the "positions", "velocities", and "accelerations" vectors for each waypoint, along with the desired `time_from_start` for each waypoint (these must be monotonically increasing).  For example, the following would be a valid motion for a 6-DoF arm:

```
rostopic pub /joint_waypoints trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
- ''
points:
- positions: [0.51, 2.09439, 2.09439, 0.01, 1.5707963, 0.01]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: []
  time_from_start: {secs: 0, nsecs: 0}
- positions: [0.01, 2.09439, 2.09439, 0.01, 1.5707963, 0.01]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: []
  time_from_start: {secs: 5, nsecs: 0}
- positions: [-0.51, 2.09439, 2.09439, 0.01, 1.5707963, 0.01]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: []
  time_from_start: {secs: 10, nsecs: 0}" 
- positions: [-0.01, 2.09439, 2.09439, 0.01, 1.5707963, 0.01]
  velocities: [0, 0, 0, 0, 0, 0]
  accelerations: [0, 0, 0, 0, 0, 0]
  effort: []
  time_from_start: {secs: 15, nsecs: 0}"
```

### `motion` action

- Use a ROS action to command the system; you can send actions on the `motion` action server topic.  For a quick test, type:

```
rostopic pub /motion/
```

and then press tab to complete the message type and create a basic yaml message on the command line.  The commanded location and color fields can be changed to cause the robot to move to a given desired (x,y,z) position or change LED colors.

Any action currently running is preempted if a subsequent action is sent, or cancelled if a message is received on one of the other channels.

# MoveIt Arm Node (`moveit_arm_node`)

## Requirements:

Set up as with the `arm_node` above.

In addition, you should have a MoveIt installation as well, and reference the appropriate HEBI moveit config. This example can then be controlled with the ROS move group interface.

## To run:

```
roslaunch hebi_cpp_api_ros_examples moveit_arm_node<configuration>.launch
```

## To command:

### `hebi_arm_controller/follow_joint_trajectory` action

- This responds to the standard `FollowJointTrajectory` action used by MoveIt.

## Feedback:

### `joint_states`

- This publishes position, velocity, and effort on the `/joint_states` channel.  You may need a robot state publisher running to transform these appropriately using `tf` for use in MoveIt.

# Base nodes (`omni_base_node`, `diff_drive_node`)

## Requirements:

Set the `families` and `names` parameters to lists of strings matching the modules on your robot. Have modules on the network matching these names.

By default, if no parameters are given, this defaults to family "HEBI" and "Wheel1", "Wheel2", "Wheel3" for the omnibase, or "Left" and "Right" for the diff drive.

For the omni base, you can also set the `publish_odom` boolean parameter; if set, this will publish x/y/theta odometry on tf broadcaster and the /odom channel.

## To run:

```
roslaunch hebi_cpp_api_ros_examples omni_base_node.launch
```

or

```
roslaunch hebi_cpp_api_ros_examples diff_drive_node.launch
```

## To command:

Either interface with the action server through code, or for a quick test, type:
```
rostopic pub /motion/
```

and then press tab to complete the message type and create a basic yaml message on the command line.  The commanded location and color fields can be changed to cause the robot to move or change LED colors.
