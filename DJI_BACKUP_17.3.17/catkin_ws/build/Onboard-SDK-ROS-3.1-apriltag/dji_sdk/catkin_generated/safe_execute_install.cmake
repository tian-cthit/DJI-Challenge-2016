execute_process(COMMAND "/home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
