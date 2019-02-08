# hebi_ros_cpp_api_examples

Examples of using HEBI components through ROS, using the HEBI C++ API.

This repository demonstrates integration of the HEBI C++ API with ROS, as well as several system-level demonstrations.  Note that HEBI also provides a the `hebi_ros` package, which is a node that wraps up many of the API function calls into ROS messages, actions, and services.  The `hebi_ros` package also provides URDF and gazebo simulation support; currently these features are not available when using the HEBI C++ API directly as the examples herein do.  However, using the API directly exposes more features of the API and more powerful functionality when communicating directly with the robot, such as the ability to create smooth motion trajectories and query and alter them _without_ sending them as commands to the robot.

## Prerequisites

You must have ROS kinetic, lunar, or melodic installed.  See http://wiki.ros.org/ROS/Installation.  We recommend the "Desktop Full" installation.  This package may work on other installations of ROS as well.

## Checkout

Create/navigate to the directory you wish to be your ROS workspace.  Then run the following commands to pull this repository and the HEBI C++ API ROS wrapper.

```
mkdir src
cd src
git clone git@github.com:HebiRobotics/hebi_cpp_api_ros.git
git clone git@github.com:HebiRobotics/hebi_cpp_api_ros_examples.git
cd ..
catkin_make
source devel/setup.sh
```

## Overview/structure

There are two folder with different types of examples.  In the "basic" folder, we provide examples that will expose particular features of the API, starting with single modules.

In the "nodes" folder, we provide full-system level demonstrations which provide a ROS node interface to a robotic system.  For example, these include an omnibase node that accepts ROS action commands for moving a HEBI omnidirectional base kit, as well as an arm node which accepts cartesian position ROS action commands.

These nodes are meant to be a starting point for your own particular functionality, or can be used as is.

The remaining folders provide the action definitions, launch files, etc. used by the examples.

## Running

To launch an example, run `roslaunch hebi_ros_cpp_api_examples <example name>`.

(if you haven't yet, you will need to run `source devel/setup.sh` first)

Depending on the example, different actions and messages are supported.  See the `README.md` files in the individual example directories for commands for each examples.
