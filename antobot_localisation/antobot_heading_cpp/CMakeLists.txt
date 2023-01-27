cmake_minimum_required(VERSION 3.0.2)
project(antobot_heading_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    tf2_geometry_msgs
    tf2
    geonav_transform
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES am_heading_cpp
   CATKIN_DEPENDS 
        roscpp
        std_msgs
        nav_msgs
        sensor_msgs
        tf2_geometry_msgs
        tf2
        geonav_transform
#  DEPENDS system_lib
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(antobot_heading
  src/antobot_heading.cpp
)

add_dependencies(antobot_heading ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(antobot_heading_node src/antobot_heading_node.cpp)

add_dependencies(antobot_heading_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(antobot_heading_node
  antobot_heading
  ${catkin_LIBRARIES}
)