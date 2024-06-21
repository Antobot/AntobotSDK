cmake_minimum_required(VERSION 2.8.3)

project(antobot_gazebo)
find_package(catkin REQUIRED COMPONENTS urdf xacro) 

catkin_package(
   #LIBRARIES keyboard_teleop
   CATKIN_DEPENDS urdf xacro
)


