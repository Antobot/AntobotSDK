cmake_minimum_required(VERSION 3.0.2)
project(antobot_ant_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
