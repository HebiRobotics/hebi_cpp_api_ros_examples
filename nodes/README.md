# Overview

These examples demonstrate use of the C++ API directly from ROS nodes. These wrap up this functionality and low-level module communication, and expose a higher-level "system" interface on ROS topics, actions, and services.

# Omnidirectional base (`omni_base_node`)

##
Requirements:

Have on the network a mobile omni-base with modules having family "Rosie" and names "Wheel1", "Wheel2", "Wheel3".

##
To run:

```
roslaunch hebi_cpp_api_ros_examples omni_base_node.launch
```

##
To command:

Either interface with the action server through code, or for a quick test, type:
```
rostopic pub /omni_base/base_motion/
```

and then press tab to complete the message type and create a basic yaml message on the command line.  The commanded location and color fields can be changed to cause the robot to move or change LED colors.
