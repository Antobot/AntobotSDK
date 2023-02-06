cmake_minimum_required(VERSION 2.8.3)
project(antobot_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
    FILES
    Float32_Array.msg
    UInt8_Array.msg
    UInt16_Array.msg
)

add_service_files(
  FILES
  camManager.srv
  softShutdown.srv
  antoRec.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)



