cmake_minimum_required(VERSION 3.0.2)
project(antobot_tel)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
)


catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)