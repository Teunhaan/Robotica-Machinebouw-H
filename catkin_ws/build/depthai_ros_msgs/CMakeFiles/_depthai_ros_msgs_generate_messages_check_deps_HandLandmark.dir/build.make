# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/student/catkin_ws/src/depth-ai-avans/depthai_ros_msgs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/catkin_ws/build/depthai_ros_msgs

# Utility rule file for _depthai_ros_msgs_generate_messages_check_deps_HandLandmark.

# Include the progress variables for this target.
include CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/progress.make

CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark:
	catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genmsg/cmake/../../../lib/genmsg/genmsg_check_deps.py depthai_ros_msgs /home/student/catkin_ws/src/depth-ai-avans/depthai_ros_msgs/msg/HandLandmark.msg geometry_msgs/Pose2D:geometry_msgs/Point

_depthai_ros_msgs_generate_messages_check_deps_HandLandmark: CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark
_depthai_ros_msgs_generate_messages_check_deps_HandLandmark: CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/build.make

.PHONY : _depthai_ros_msgs_generate_messages_check_deps_HandLandmark

# Rule to build all files generated by this target.
CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/build: _depthai_ros_msgs_generate_messages_check_deps_HandLandmark

.PHONY : CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/build

CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/cmake_clean.cmake
.PHONY : CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/clean

CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/depend:
	cd /home/student/catkin_ws/build/depthai_ros_msgs && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/catkin_ws/src/depth-ai-avans/depthai_ros_msgs /home/student/catkin_ws/src/depth-ai-avans/depthai_ros_msgs /home/student/catkin_ws/build/depthai_ros_msgs /home/student/catkin_ws/build/depthai_ros_msgs /home/student/catkin_ws/build/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_HandLandmark.dir/depend

