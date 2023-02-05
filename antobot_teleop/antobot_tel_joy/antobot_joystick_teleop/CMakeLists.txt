cmake_minimum_required(VERSION 2.8.3)
project(antobot_joystick_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  joy
)

catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
