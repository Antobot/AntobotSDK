cmake_minimum_required(VERSION 2.8.3)
project(antobot_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  antobot_msgs
  std_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    std_msgs
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(antobot_base src/antobot_control.cpp)
target_link_libraries(antobot_base
	${antobot_base_LIBRARIES}
	${catkin_LIBRARIES})
add_executable(antobot_control_node src/antobot_control_node.cpp)
add_dependencies(antobot_control_node antobot_base antobot_msgs)

target_link_libraries(antobot_control_node
	antobot_base
	${catkin_LIBRARIES})


