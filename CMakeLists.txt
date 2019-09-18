cmake_minimum_required(VERSION 3.1.3)
project(hebi_cpp_api_examples)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  genmsg
  actionlib_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  control_msgs
  trajectory_msgs
  actionlib
  hebi_cpp_api
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  ArmMotion.action
  BaseMotion.action
)

add_message_files(
  FILES
  TargetWaypoints.msg
  SaveWaypoint.msg
  StartPath.msg
  EndPath.msg
  Playback.msg
  OffsetPlayback.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  trajectory_msgs
  std_msgs
  nav_msgs
  actionlib_msgs
  control_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp
  CATKIN_DEPENDS actionlib
  CATKIN_DEPENDS std_msgs
  CATKIN_DEPENDS nav_msgs
  CATKIN_DEPENDS geometry_msgs
  CATKIN_DEPENDS tf
  CATKIN_DEPENDS trajectory_msgs
  CATKIN_DEPENDS control_msgs
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS hebi_cpp_api
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  .
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  config/a-2085-04_params.yaml
  config/a-2085-05_params.yaml
  config/a-2085-06_params.yaml
  launch/arm_node.launch
  launch/move_it_node.launch
  launch/diff_drive.launch
  launch/omni_base.launch
  launch/omni_base_control.launch
  launch/teach_repeat_6_dof.launch
  kits/rosie/rosie_teach_repeat.launch

  gains/omni_base_gains.xml

  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## I don't think this is the _right_ way to do this, but it works...
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/gains/omni_base_gains.xml
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

# Different nodes for the arm kits

add_executable(omni_base_node nodes/omni_base_node.cpp robot/omni_base.cpp util/odom_publisher.cpp)
add_dependencies(omni_base_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_base_node ${catkin_LIBRARIES} )

add_executable(omni_base_controller_node nodes/omni_base_controller_node.cpp util/mobile_io.cpp)
add_dependencies(omni_base_controller_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_base_controller_node ${catkin_LIBRARIES} )

add_executable(rosie_teach_repeat_controller_node kits/rosie/teach_repeat_controller_node.cpp util/mobile_io.cpp)
add_dependencies(rosie_teach_repeat_controller_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosie_teach_repeat_controller_node ${catkin_LIBRARIES} )

add_executable(diff_drive_node nodes/diff_drive_node.cpp robot/diff_drive.cpp)
add_dependencies(diff_drive_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_drive_node ${catkin_LIBRARIES} )

# Different nodes for the arm kits

add_executable(arm_teach_repeat_node src/kits/arm/arm_teach_repeat_node.cpp robot/arm.cpp robot/kinematics_helper.cpp util/path.cpp util/waypoint.cpp)
add_dependencies(arm_teach_repeat_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_teach_repeat_node ${catkin_LIBRARIES} )

add_executable(arm_node src/kits/arm/arm_node.cpp robot/arm.cpp robot/kinematics_helper.cpp)
add_dependencies(arm_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_node ${catkin_LIBRARIES} )

add_executable(arm_moveit_node src/kits/arm/arm_moveit_node.cpp robot/arm.cpp robot/kinematics_helper.cpp)
add_dependencies(arm_moveit_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_moveit_node ${catkin_LIBRARIES} )

add_executable(gripper_node nodes/gripper_node.cpp)
add_dependencies(gripper_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(gripper_node ${catkin_LIBRARIES} )

