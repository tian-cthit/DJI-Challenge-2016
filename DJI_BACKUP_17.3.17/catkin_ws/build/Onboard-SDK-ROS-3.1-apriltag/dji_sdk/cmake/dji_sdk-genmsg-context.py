# generated from genmsg/cmake/pkg-genmsg.context.in

messages_str = "/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/Acceleration.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/AttitudeQuaternion.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/Compass.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/FlightControlInfo.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/Gimbal.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/GlobalPosition.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/LocalPosition.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/PowerStatus.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/RCChannels.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/Velocity.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/Waypoint.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/WaypointList.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/TransparentTransmissionData.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/TimeStamp.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionPushInfo.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionWaypointAction.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionWaypoint.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionWaypointTask.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionHotpointTask.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionFollowmeTask.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionFollowmeTarget.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionStatusWaypoint.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionStatusHotpoint.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionStatusFollowme.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionStatusOther.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionEventWpUpload.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionEventWpAction.msg;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg/MissionEventWpReach.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationAction.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationActionGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationActionResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationActionFeedback.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/GlobalPositionNavigationFeedback.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationAction.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationActionGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationActionResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationActionFeedback.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/LocalPositionNavigationFeedback.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationAction.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationActionGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationActionResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationActionFeedback.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/WaypointNavigationFeedback.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskAction.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskActionGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskActionResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskActionFeedback.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskGoal.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskResult.msg;/home/tx/catkin_ws/devel/share/dji_sdk/msg/DroneTaskFeedback.msg"
services_str = "/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/Activation.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/AttitudeControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/CameraActionControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/DroneTaskControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/GimbalAngleControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/GimbalSpeedControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/GlobalPositionControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/LocalPositionControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/SDKPermissionControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/SendDataToRemoteDevice.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/VelocityControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/VersionCheck.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/DroneArmControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/SyncFlagControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MessageFrequencyControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/VirtualRCEnableControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/VirtualRCDataControl.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionStart.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionPause.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionCancel.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionWpUpload.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionWpSetSpeed.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionWpGetSpeed.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionWpDownload.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionHpUpload.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionHpDownload.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionHpSetSpeed.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionHpSetRadius.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionHpResetYaw.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionFmUpload.srv;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/srv/MissionFmSetTarget.srv"
pkg_name = "dji_sdk"
dependencies_str = "geometry_msgs;nav_msgs;std_msgs;actionlib_msgs"
langs = "gencpp;genlisp;genpy"
dep_include_paths_str = "dji_sdk;/home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk/msg;dji_sdk;/home/tx/catkin_ws/devel/share/dji_sdk/msg;geometry_msgs;/opt/ros/indigo/share/geometry_msgs/cmake/../msg;nav_msgs;/opt/ros/indigo/share/nav_msgs/cmake/../msg;std_msgs;/opt/ros/indigo/share/std_msgs/cmake/../msg;actionlib_msgs;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg"
PYTHON_EXECUTABLE = "/usr/bin/python"
package_has_static_sources = 'TRUE' == 'TRUE'
genmsg_check_deps_script = "/opt/ros/indigo/share/genmsg/cmake/../../../lib/genmsg/genmsg_check_deps.py"
