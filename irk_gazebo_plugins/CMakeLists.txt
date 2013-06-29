cmake_minimum_required(VERSION 2.4.6)
project(irk_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
  roscpp gazebo_ros_control gazebo_ros)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros_control gazebo_ros
  )

# Build RobotSim Interface
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/robot_sim_mtm.cpp)
