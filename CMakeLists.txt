cmake_minimum_required(VERSION 3.0.2)
project(fake_laser_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  turtlesim
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Shape.msg
)

add_service_files(
  FILES
  SpawnShape.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  turtlesim
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_laser_sim
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/env_server_node.py
  scripts/object_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

