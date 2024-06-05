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
CMAKE_SOURCE_DIR = /home/student/catkin_ws/src/depth-ai-avans/depthai_examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/catkin_ws/build/depthai_examples

# Include any dependencies generated for this target.
include CMakeFiles/nodelet_stereo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/nodelet_stereo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/nodelet_stereo.dir/flags.make

CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o: CMakeFiles/nodelet_stereo.dir/flags.make
CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o: /home/student/catkin_ws/src/depth-ai-avans/depthai_examples/src/stereo_nodelet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/student/catkin_ws/build/depthai_examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o -c /home/student/catkin_ws/src/depth-ai-avans/depthai_examples/src/stereo_nodelet.cpp

CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/student/catkin_ws/src/depth-ai-avans/depthai_examples/src/stereo_nodelet.cpp > CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.i

CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/student/catkin_ws/src/depth-ai-avans/depthai_examples/src/stereo_nodelet.cpp -o CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.s

CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.requires:

.PHONY : CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.requires

CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.provides: CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.requires
	$(MAKE) -f CMakeFiles/nodelet_stereo.dir/build.make CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.provides.build
.PHONY : CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.provides

CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.provides.build: CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o


# Object files for target nodelet_stereo
nodelet_stereo_OBJECTS = \
"CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o"

# External object files for target nodelet_stereo
nodelet_stereo_EXTERNAL_OBJECTS =

/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: CMakeFiles/nodelet_stereo.dir/build.make
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /home/student/catkin_ws/devel/.private/depthai_bridge/lib/libdepthai_bridge.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libcamera_info_manager.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libcamera_calibration_parsers.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libimage_transport.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libcv_bridge.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libmessage_filters.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libnodeletlib.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libbondcpp.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libuuid.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libclass_loader.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/libPocoFoundation.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libroslib.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/librospack.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libroscpp.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/librosconsole.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/librostime.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /opt/ros/melodic/lib/libcpp_common.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/local/lib/libdepthai-core.so
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
/home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so: CMakeFiles/nodelet_stereo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/student/catkin_ws/build/depthai_examples/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nodelet_stereo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/nodelet_stereo.dir/build: /home/student/catkin_ws/devel/.private/depthai_examples/lib/libnodelet_stereo.so

.PHONY : CMakeFiles/nodelet_stereo.dir/build

CMakeFiles/nodelet_stereo.dir/requires: CMakeFiles/nodelet_stereo.dir/src/stereo_nodelet.cpp.o.requires

.PHONY : CMakeFiles/nodelet_stereo.dir/requires

CMakeFiles/nodelet_stereo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/nodelet_stereo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/nodelet_stereo.dir/clean

CMakeFiles/nodelet_stereo.dir/depend:
	cd /home/student/catkin_ws/build/depthai_examples && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/catkin_ws/src/depth-ai-avans/depthai_examples /home/student/catkin_ws/src/depth-ai-avans/depthai_examples /home/student/catkin_ws/build/depthai_examples /home/student/catkin_ws/build/depthai_examples /home/student/catkin_ws/build/depthai_examples/CMakeFiles/nodelet_stereo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/nodelet_stereo.dir/depend

