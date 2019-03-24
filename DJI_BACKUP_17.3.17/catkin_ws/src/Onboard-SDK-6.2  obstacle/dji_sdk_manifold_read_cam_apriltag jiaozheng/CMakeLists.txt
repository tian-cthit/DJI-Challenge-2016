cmake_minimum_required(VERSION 2.8.3)
project(dji_sdk_read_cam)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages
catkin_package()
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  #opencv2
  roscpp
  sensor_msgs
  std_msgs
  dji_sdk
)

set(POD_NAME apriltags)
include(cmake/pods.cmake)
link_libraries(apriltags)
file(GLOB SOURCE_FILES "src/*.cc")
include_directories(AprilTags . /opt/local/include)
add_library(apriltags ${SOURCE_FILES})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(apriltags ${OpenCV_LIBS}) #-pg) #-fopenmp)
pods_use_pkg_config_packages(apriltags eigen3)

if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(apriltags -L/opt/local/lib/) # MacPorts special treatment...
else (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  pods_use_pkg_config_packages(apriltags libv4l2)
endif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

pods_install_libraries(apriltags)

file(GLOB header_files "AprilTags/*.h")
pods_install_headers(${header_files} DESTINATION AprilTags/)

pods_install_pkg_config_file(apriltags
    LIBS -lapriltags
    REQUIRES eigen3 opencv2
    VERSION 1.0.0)
#add_subdirectory(example)


set(CMAKE_CXX_FLAGS "-std=c++11 -O3 ${CMAKE_CXX_FLAGS}")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(dji_sdk_read_cam src/nv_cam.cpp)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)
target_link_libraries(
  dji_sdk_read_cam
  dcam
  ${catkin_LIBRARIES})
add_dependencies(dji_sdk_read_cam dji_sdk_generate_messages_cpp)
