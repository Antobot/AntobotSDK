cmake_minimum_required(VERSION 3.0.2)
project(antobot_ekf)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)


catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
