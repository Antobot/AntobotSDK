cmake_minimum_required(VERSION 3.0.2)
project(antobot_heading)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    std_srvs
    nav_msgs
    sensor_msgs
    tf2_geometry_msgs
    tf2
    geonav_transform
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS 
        roscpp
        std_msgs
        std_srvs
        nav_msgs
        sensor_msgs
        tf2_geometry_msgs
        tf2
        geonav_transform
)

###########
## Build ##
###########

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(antobot_heading
  src/antobot_heading.cpp
)

add_dependencies(antobot_heading ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(antobot_heading_node src/antobot_heading_node.cpp)

add_dependencies(antobot_heading_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(antobot_heading_node
  antobot_heading
  ${catkin_LIBRARIES}
)

