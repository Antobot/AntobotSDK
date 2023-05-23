execute_process(COMMAND "/home/antobot/catkin_ws/src/AntobotSDK/antobot_datamanager/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/antobot/catkin_ws/src/AntobotSDK/antobot_datamanager/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
