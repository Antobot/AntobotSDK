cmake_minimum_required(VERSION 3.0.2)
project(antobot_gps)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
   src/gps_config.py
   src/antobot_gps_node.py
   src/antobot_gps/ublox_gps.py
   src/antobot_gps/core.py
   src/antobot_gps/sparkfun_predefines.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
