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
CMAKE_SOURCE_DIR = /home/luigi/projects/apriltag

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luigi/projects/apriltag

# Include any dependencies generated for this target.
include CMakeFiles/opencv_demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/opencv_demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/opencv_demo.dir/flags.make

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o: CMakeFiles/opencv_demo.dir/flags.make
CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o: example/opencv_demo.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luigi/projects/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o -c /home/luigi/projects/apriltag/example/opencv_demo.cc

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luigi/projects/apriltag/example/opencv_demo.cc > CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.i

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luigi/projects/apriltag/example/opencv_demo.cc -o CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.s

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires:

.PHONY : CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires
	$(MAKE) -f CMakeFiles/opencv_demo.dir/build.make CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides.build
.PHONY : CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides.build: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o


CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o: CMakeFiles/opencv_demo.dir/flags.make
CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o: zmq_layer/Publisher.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luigi/projects/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o -c /home/luigi/projects/apriltag/zmq_layer/Publisher.cpp

CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luigi/projects/apriltag/zmq_layer/Publisher.cpp > CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.i

CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luigi/projects/apriltag/zmq_layer/Publisher.cpp -o CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.s

CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.requires:

.PHONY : CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.requires

CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.provides: CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.requires
	$(MAKE) -f CMakeFiles/opencv_demo.dir/build.make CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.provides.build
.PHONY : CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.provides

CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.provides.build: CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o


CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o: CMakeFiles/opencv_demo.dir/flags.make
CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o: zmq_layer/Subscriber.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luigi/projects/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o -c /home/luigi/projects/apriltag/zmq_layer/Subscriber.cpp

CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luigi/projects/apriltag/zmq_layer/Subscriber.cpp > CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.i

CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luigi/projects/apriltag/zmq_layer/Subscriber.cpp -o CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.s

CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.requires:

.PHONY : CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.requires

CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.provides: CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.requires
	$(MAKE) -f CMakeFiles/opencv_demo.dir/build.make CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.provides.build
.PHONY : CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.provides

CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.provides.build: CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o


CMakeFiles/opencv_demo.dir/mathUtils.cpp.o: CMakeFiles/opencv_demo.dir/flags.make
CMakeFiles/opencv_demo.dir/mathUtils.cpp.o: mathUtils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/luigi/projects/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/opencv_demo.dir/mathUtils.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_demo.dir/mathUtils.cpp.o -c /home/luigi/projects/apriltag/mathUtils.cpp

CMakeFiles/opencv_demo.dir/mathUtils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_demo.dir/mathUtils.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/luigi/projects/apriltag/mathUtils.cpp > CMakeFiles/opencv_demo.dir/mathUtils.cpp.i

CMakeFiles/opencv_demo.dir/mathUtils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_demo.dir/mathUtils.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/luigi/projects/apriltag/mathUtils.cpp -o CMakeFiles/opencv_demo.dir/mathUtils.cpp.s

CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.requires:

.PHONY : CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.requires

CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.provides: CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.requires
	$(MAKE) -f CMakeFiles/opencv_demo.dir/build.make CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.provides.build
.PHONY : CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.provides

CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.provides.build: CMakeFiles/opencv_demo.dir/mathUtils.cpp.o


# Object files for target opencv_demo
opencv_demo_OBJECTS = \
"CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o" \
"CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o" \
"CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o" \
"CMakeFiles/opencv_demo.dir/mathUtils.cpp.o"

# External object files for target opencv_demo
opencv_demo_EXTERNAL_OBJECTS =

opencv_demo: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o
opencv_demo: CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o
opencv_demo: CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o
opencv_demo: CMakeFiles/opencv_demo.dir/mathUtils.cpp.o
opencv_demo: CMakeFiles/opencv_demo.dir/build.make
opencv_demo: libapriltag.so.3.1.0
opencv_demo: /usr/local/lib/libopencv_dnn.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_gapi.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_highgui.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_ml.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_objdetect.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_photo.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_stitching.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_video.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_videoio.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_calib3d.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_features2d.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_flann.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_imgproc.so.4.5.0
opencv_demo: /usr/local/lib/libopencv_core.so.4.5.0
opencv_demo: CMakeFiles/opencv_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/luigi/projects/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable opencv_demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencv_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/opencv_demo.dir/build: opencv_demo

.PHONY : CMakeFiles/opencv_demo.dir/build

CMakeFiles/opencv_demo.dir/requires: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires
CMakeFiles/opencv_demo.dir/requires: CMakeFiles/opencv_demo.dir/zmq_layer/Publisher.cpp.o.requires
CMakeFiles/opencv_demo.dir/requires: CMakeFiles/opencv_demo.dir/zmq_layer/Subscriber.cpp.o.requires
CMakeFiles/opencv_demo.dir/requires: CMakeFiles/opencv_demo.dir/mathUtils.cpp.o.requires

.PHONY : CMakeFiles/opencv_demo.dir/requires

CMakeFiles/opencv_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/opencv_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/opencv_demo.dir/clean

CMakeFiles/opencv_demo.dir/depend:
	cd /home/luigi/projects/apriltag && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luigi/projects/apriltag /home/luigi/projects/apriltag /home/luigi/projects/apriltag /home/luigi/projects/apriltag /home/luigi/projects/apriltag/CMakeFiles/opencv_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/opencv_demo.dir/depend

