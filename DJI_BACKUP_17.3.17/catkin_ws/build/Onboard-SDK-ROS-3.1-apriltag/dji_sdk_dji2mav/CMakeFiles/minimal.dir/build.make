# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tx/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tx/catkin_ws/build

# Include any dependencies generated for this target.
include Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/depend.make

# Include the progress variables for this target.
include Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/progress.make

# Include the compile flags for this target's objects.
include Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/flags.make

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o: Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/flags.make
Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o: /home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/src/minimal.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tx/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o"
	cd /home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/minimal.dir/src/minimal.cpp.o -c /home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/src/minimal.cpp

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/minimal.dir/src/minimal.cpp.i"
	cd /home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/src/minimal.cpp > CMakeFiles/minimal.dir/src/minimal.cpp.i

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/minimal.dir/src/minimal.cpp.s"
	cd /home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/src/minimal.cpp -o CMakeFiles/minimal.dir/src/minimal.cpp.s

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.requires:
.PHONY : Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.requires

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.provides: Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.requires
	$(MAKE) -f Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/build.make Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.provides.build
.PHONY : Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.provides

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.provides.build: Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o

# Object files for target minimal
minimal_OBJECTS = \
"CMakeFiles/minimal.dir/src/minimal.cpp.o"

# External object files for target minimal
minimal_EXTERNAL_OBJECTS =

/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/build.make
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/libactionlib.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /home/tx/catkin_ws/devel/lib/libdji_sdk_lib.a
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/libroscpp.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/librosconsole.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/liblog4cxx.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/librostime.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /opt/ros/indigo/lib/libcpp_common.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal: Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal"
	cd /home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/minimal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/build: /home/tx/catkin_ws/devel/lib/dji_sdk_dji2mav/minimal
.PHONY : Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/build

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/requires: Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/src/minimal.cpp.o.requires
.PHONY : Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/requires

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/clean:
	cd /home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav && $(CMAKE_COMMAND) -P CMakeFiles/minimal.dir/cmake_clean.cmake
.PHONY : Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/clean

Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/depend:
	cd /home/tx/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tx/catkin_ws/src /home/tx/catkin_ws/src/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav /home/tx/catkin_ws/build /home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav /home/tx/catkin_ws/build/Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Onboard-SDK-ROS-3.1-apriltag/dji_sdk_dji2mav/CMakeFiles/minimal.dir/depend

