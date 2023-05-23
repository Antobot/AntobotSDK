# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "antobot_msgs: 3 messages, 3 services")

set(MSG_I_FLAGS "-Iantobot_msgs:/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(antobot_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg" NAME_WE)
add_custom_target(_antobot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "antobot_msgs" "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg" ""
)

get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg" NAME_WE)
add_custom_target(_antobot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "antobot_msgs" "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg" ""
)

get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg" NAME_WE)
add_custom_target(_antobot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "antobot_msgs" "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg" ""
)

get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv" NAME_WE)
add_custom_target(_antobot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "antobot_msgs" "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv" ""
)

get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv" NAME_WE)
add_custom_target(_antobot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "antobot_msgs" "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv" ""
)

get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv" NAME_WE)
add_custom_target(_antobot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "antobot_msgs" "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
)
_generate_msg_cpp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
)
_generate_msg_cpp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
)

### Generating Services
_generate_srv_cpp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
)
_generate_srv_cpp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
)
_generate_srv_cpp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
)

### Generating Module File
_generate_module_cpp(antobot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(antobot_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(antobot_msgs_generate_messages antobot_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_cpp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_cpp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_cpp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_cpp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_cpp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_cpp _antobot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(antobot_msgs_gencpp)
add_dependencies(antobot_msgs_gencpp antobot_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS antobot_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
)
_generate_msg_eus(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
)
_generate_msg_eus(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
)

### Generating Services
_generate_srv_eus(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
)
_generate_srv_eus(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
)
_generate_srv_eus(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
)

### Generating Module File
_generate_module_eus(antobot_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(antobot_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(antobot_msgs_generate_messages antobot_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_eus _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_eus _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_eus _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_eus _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_eus _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_eus _antobot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(antobot_msgs_geneus)
add_dependencies(antobot_msgs_geneus antobot_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS antobot_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
)
_generate_msg_lisp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
)
_generate_msg_lisp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
)

### Generating Services
_generate_srv_lisp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
)
_generate_srv_lisp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
)
_generate_srv_lisp(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
)

### Generating Module File
_generate_module_lisp(antobot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(antobot_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(antobot_msgs_generate_messages antobot_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_lisp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_lisp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_lisp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_lisp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_lisp _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_lisp _antobot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(antobot_msgs_genlisp)
add_dependencies(antobot_msgs_genlisp antobot_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS antobot_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
)
_generate_msg_nodejs(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
)
_generate_msg_nodejs(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
)

### Generating Services
_generate_srv_nodejs(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
)
_generate_srv_nodejs(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
)
_generate_srv_nodejs(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
)

### Generating Module File
_generate_module_nodejs(antobot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(antobot_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(antobot_msgs_generate_messages antobot_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_nodejs _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_nodejs _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_nodejs _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_nodejs _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_nodejs _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_nodejs _antobot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(antobot_msgs_gennodejs)
add_dependencies(antobot_msgs_gennodejs antobot_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS antobot_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
)
_generate_msg_py(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
)
_generate_msg_py(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
)

### Generating Services
_generate_srv_py(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
)
_generate_srv_py(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
)
_generate_srv_py(antobot_msgs
  "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
)

### Generating Module File
_generate_module_py(antobot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(antobot_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(antobot_msgs_generate_messages antobot_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/Float32_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_py _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt8_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_py _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/msg/UInt16_Array.msg" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_py _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/camManager.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_py _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/softShutdown.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_py _antobot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/antobot/catkin_ws/src/AntobotSDK/antobot_msgs/srv/antoRec.srv" NAME_WE)
add_dependencies(antobot_msgs_generate_messages_py _antobot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(antobot_msgs_genpy)
add_dependencies(antobot_msgs_genpy antobot_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS antobot_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/antobot_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(antobot_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/antobot_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(antobot_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/antobot_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(antobot_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/antobot_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(antobot_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/antobot_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(antobot_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
