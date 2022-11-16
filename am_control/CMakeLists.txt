cmake_minimum_required(VERSION 2.8.3)
project(am_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
#  rospy
  control_msgs
  trajectory_msgs
  actionlib
  pluginlib
  transmission_interface
  urdf
  control_toolbox
  joint_limits_interface
  # anto_bridge
  anto_bridge_msgs
  serial
  std_msgs
  message_generation
)

# add_message_files(
#     FILES
#     UInt8_Array.msg
#     Bool.msg
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    hardware_interface
    controller_manager
    roscpp
 #   rospy
    control_msgs
    trajectory_msgs
    pluginlib
    transmission_interface
    urdf
    control_toolbox
    joint_limits_interface
    serial
    std_msgs
   # LIBRARIES 
     # anto_bridge
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

add_library(am_control src/am_control.cpp)
target_link_libraries(am_control
	${am_control_LIBRARIES}
	${catkin_LIBRARIES})
add_executable(am_control_node src/am_control_node.cpp)
add_dependencies(am_control_node am_control anto_bridge_msgs)

target_link_libraries(am_control_node
	am_control
	${catkin_LIBRARIES})


