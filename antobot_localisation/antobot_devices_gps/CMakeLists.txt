cmake_minimum_required(VERSION 3.0.2)
project(antobot_devices_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS 
    std_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(antobot_devices_gps_node src/utils/field_survey.cpp)
target_link_libraries(antobot_devices_gps_node
	${catkin_LIBRARIES})
