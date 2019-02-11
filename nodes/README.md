# Overview

These examples demonstrate use of the C++ API directly from ROS nodes. These wrap up this functionality and low-level module communication, and expose a higher-level "system" interface on ROS topics, actions, and services.

**Important: to run any of these commands, you will need to run `source devel/setup.sh` from your catkin workspace first**

# Arm (`arm_node`, `arm_node_action`)

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

### `arm_node`

Note: you can use `rostopic pub` on the command line for the following `publish` lines; press tab to see the available channels, message types, and to create a basic filled out YAML message that you can edit.  For more advanced editing, copy the message into a text editor and paste back the full message.

- Publish `geometry_msgs::Point` messages on the `offset_target` channel to jog the end effector in a certain cartesian direction relative to the base. (The initial "offset" value is taken from the actuator's current position.)

or

- Publish `geometry_msgs::Point` messages on the `set_target` channel to set the end effector cartesian position relative to the base.

or

- Publish `hebi_cpp_api_ros_examples::TargetWaypoint` messages on the `cartesian_waypoints` channel to command a series of cartesian waypoints for the arm to pass through.

or

- Use a ROS action to command the system; you can send actions on the `motion` action server topic.  For a quick test, type:

```
rostopic pub /motion/
```

and then press tab to complete the message type and create a basic yaml message on the command line.  The commanded location and color fields can be changed to cause the robot to move to a given desired (x,y,z) position or change LED colors.

Any action currently running is preempted if a subsequent action is sent, or cancelled if a message is received on one of the other channels.

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
