################################################################################
# Set minimum required version of CMake, project name, and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(rur_navigation)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  rospy
  std_msgs
  move_base_msgs
  actionlib
  message_runtime
)

################################################################################
# Declare ROS messages, services, and actions
################################################################################
add_action_files(
  FILES
  Order.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime rospy std_msgs move_base_msgs actionlib
)

################################################################################
# Ensure message headers are built before other targets
################################################################################
add_dependencies(${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

################################################################################
# Install required files
################################################################################
install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/butler_order_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

################################################################################
# Test
################################################################################

