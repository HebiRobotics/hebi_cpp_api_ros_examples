# Overview

These examples demonstrate use of the C++ API directly from ROS nodes. These wrap up this functionality and low-level module communication, and expose a higher-level "system" interface on ROS topics, actions, and services.

# Arm (`arm_node`, `arm_node_action`)

## Requirements:

Set the `families` and `names` parameters to lists of strings, ordered from proximal to distal, of your robot. Have modules on the network matching these names.

Note: if all families are the same, this parameter can be a length one vector.

Note: the `arm_node_action` currently requires additional parameters to be set for the location of the gains, HRDF file, and home position of the system.  These will be added to the launch file example in an upcoming commit.

## To run:

```
roslaunch hebi_cpp_api_ros_examples arm_node.launch
```

or

```
roslaunch hebi_cpp_api_ros_examples arm_node_action.launch
```

## To command:

### `arm_node`

- Publish `geometry_msgs::Point` messages on the `keys/cmd_vel` channel to jog the end effector in a certain cartesian direction relative to the base.

or

- Publish `hebi_cpp_api_ros_examples::TargetWaypoint` messages on the `cartesian_waypoints` channel to command a series of cartesian waypoints for the arm to pass through.

Note: you can use `rostopic pub` on the command line for this; press tab to see the available channels, message types, and to create a basic filled out YAML message that you can edit.

### `arm_node_action`

- This uses ros actions to command the system; you can send actions on the `motion` action server topic.  For a quick test, type:
```
rostopic pub /motion/
```

and then press tab to complete the message type and create a basic yaml message on the command line.  The commanded location and color fields can be changed to cause the robot to move to a given desired (x,y,z) position or change LED colors.

# Base nodes (`omni_base_node`, `diff_drive_node`)

## Requirements:

Set the `families` and `names` parameters to lists of strings matching the modules on your robot. Have modules on the network matching these names.

By default, if no parameters are given, this defaults to family "HEBI" and "Wheel1", "Wheel2", "Wheel3" for the omnibase, or "Left" and "Right" for the diff drive.

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
