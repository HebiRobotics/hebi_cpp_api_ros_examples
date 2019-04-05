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
  geometry_msgs
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
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp
  CATKIN_DEPENDS actionlib
  CATKIN_DEPENDS std_msgs
  CATKIN_DEPENDS geometry_msgs
  CATKIN_DEPENDS trajectory_msgs
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
  launch/base.launch
  gains/omni_base_gains.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## I don't think this is the _right_ way to do this, but it works...
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/gains/omni_base_gains.xml
      DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(omni_base_node nodes/omni_base_node.cpp robot/omni_base.cpp)
add_dependencies(omni_base_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_base_node ${catkin_LIBRARIES} )

add_executable(diff_drive_node nodes/diff_drive_node.cpp robot/diff_drive.cpp)
add_dependencies(diff_drive_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_drive_node ${catkin_LIBRARIES} )

add_executable(teach_repeat_node nodes/teach_repeat_node.cpp robot/arm.cpp robot/arm_kinematics.cpp robot/arm_trajectory.cpp util/path.cpp util/waypoint.cpp)
add_dependencies(teach_repeat_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(teach_repeat_node ${catkin_LIBRARIES} )

add_executable(arm_node nodes/arm_node.cpp robot/arm.cpp robot/arm_kinematics.cpp robot/arm_trajectory.cpp)
add_dependencies(arm_node hebi_cpp_api_examples_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_node ${catkin_LIBRARIES} )
