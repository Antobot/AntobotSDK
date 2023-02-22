cmake_minimum_required(VERSION 3.0.2)
project(antobot_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

