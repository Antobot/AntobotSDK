cmake_minimum_required(VERSION 3.0.2)
project(antobot_tel_key)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


