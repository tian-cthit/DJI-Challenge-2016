# Install script for directory: /home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/tx/catkin_ws/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  INCLUDE("/home/tx/catkin_ws/build/Onboard-SDK-6.2  obstacle/dji_sdk/catkin_generated/safe_execute_install.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/Acceleration.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/AttitudeQuaternion.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/Compass.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/FlightControlInfo.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/Gimbal.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/GlobalPosition.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/LocalPosition.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/PowerStatus.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/RCChannels.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/Velocity.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/Waypoint.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/WaypointList.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/TransparentTransmissionData.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/TimeStamp.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionPushInfo.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionWaypointAction.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionWaypoint.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionWaypointTask.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionHotpointTask.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionFollowmeTask.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionFollowmeTarget.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionStatusWaypoint.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionStatusHotpoint.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionStatusFollowme.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionStatusOther.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionEventWpUpload.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionEventWpAction.msg"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg/MissionEventWpReach.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/srv" TYPE FILE FILES
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/Activation.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/AttitudeControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/CameraActionControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/DroneTaskControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/GimbalAngleControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/GimbalSpeedControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/GlobalPositionControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/LocalPositionControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/SDKPermissionControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/SendDataToRemoteDevice.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/VelocityControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/VersionCheck.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/DroneArmControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/SyncFlagControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MessageFrequencyControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/VirtualRCEnableControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/VirtualRCDataControl.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionStart.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionPause.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionCancel.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionWpUpload.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionWpSetSpeed.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionWpGetSpeed.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionWpDownload.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionHpUpload.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionHpDownload.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionHpSetSpeed.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionHpSetRadius.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionHpResetYaw.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionFmUpload.srv"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/srv/MissionFmSetTarget.srv"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/action" TYPE FILE FILES
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/action/GlobalPositionNavigation.action"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/action/LocalPositionNavigation.action"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/action/WaypointNavigation.action"
    "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/action/DroneTask.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationAction.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationActionGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationActionResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationActionFeedback.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationAction.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationActionGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationActionResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationActionFeedback.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationAction.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationActionGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationActionResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationActionFeedback.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/msg" TYPE FILE FILES
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskAction.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskActionGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskActionResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskActionFeedback.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskGoal.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskResult.msg"
    "/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/home/tx/catkin_ws/build/Onboard-SDK-6.2  obstacle/dji_sdk/catkin_generated/installspace/dji_sdk-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/tx/catkin_ws/devel/include/dji_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/tx/catkin_ws/devel/share/common-lisp/ros/dji_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/tx/catkin_ws/devel/lib/python2.7/dist-packages/dji_sdk")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/tx/catkin_ws/devel/lib/python2.7/dist-packages/dji_sdk" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/tx/catkin_ws/devel/lib/python2.7/dist-packages/dji_sdk" FILES_MATCHING REGEX "/home/tx/catkin_ws/devel/lib/python2.7/dist-packages/dji_sdk/.+/__init__.pyc?$")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tx/catkin_ws/build/Onboard-SDK-6.2  obstacle/dji_sdk/catkin_generated/installspace/dji_sdk.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES "/home/tx/catkin_ws/build/Onboard-SDK-6.2  obstacle/dji_sdk/catkin_generated/installspace/dji_sdk-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/cmake" TYPE FILE FILES
    "/home/tx/catkin_ws/build/Onboard-SDK-6.2  obstacle/dji_sdk/catkin_generated/installspace/dji_sdkConfig.cmake"
    "/home/tx/catkin_ws/build/Onboard-SDK-6.2  obstacle/dji_sdk/catkin_generated/installspace/dji_sdkConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk" TYPE FILE FILES "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/dji_sdk" TYPE DIRECTORY FILES "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/include/dji_sdk/")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dji_sdk" TYPE EXECUTABLE FILES "/home/tx/catkin_ws/devel/lib/dji_sdk/dji_sdk_node")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/dji_sdk/dji_sdk_node")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dji_sdk/launch" TYPE DIRECTORY FILES "/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/launch")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

