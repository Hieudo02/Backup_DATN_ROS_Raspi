# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/pi/datn_navbot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/datn_navbot/build

# Include any dependencies generated for this target.
include localization_data_pub/CMakeFiles/sendUartSTM32.dir/depend.make

# Include the progress variables for this target.
include localization_data_pub/CMakeFiles/sendUartSTM32.dir/progress.make

# Include the compile flags for this target's objects.
include localization_data_pub/CMakeFiles/sendUartSTM32.dir/flags.make

localization_data_pub/CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.o: localization_data_pub/CMakeFiles/sendUartSTM32.dir/flags.make
localization_data_pub/CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.o: /home/pi/datn_navbot/src/localization_data_pub/src/sendUartSTM32.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/datn_navbot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object localization_data_pub/CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.o"
	cd /home/pi/datn_navbot/build/localization_data_pub && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.o -c /home/pi/datn_navbot/src/localization_data_pub/src/sendUartSTM32.cpp

localization_data_pub/CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.i"
	cd /home/pi/datn_navbot/build/localization_data_pub && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/datn_navbot/src/localization_data_pub/src/sendUartSTM32.cpp > CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.i

localization_data_pub/CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.s"
	cd /home/pi/datn_navbot/build/localization_data_pub && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/datn_navbot/src/localization_data_pub/src/sendUartSTM32.cpp -o CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.s

# Object files for target sendUartSTM32
sendUartSTM32_OBJECTS = \
"CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.o"

# External object files for target sendUartSTM32
sendUartSTM32_EXTERNAL_OBJECTS =

/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: localization_data_pub/CMakeFiles/sendUartSTM32.dir/src/sendUartSTM32.cpp.o
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: localization_data_pub/CMakeFiles/sendUartSTM32.dir/build.make
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libtf.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libtf2_ros.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libactionlib.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libmessage_filters.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libroscpp.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/librosconsole.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libtf2.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/librostime.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libcpp_common.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: /opt/ros/noetic/lib/libserial.so
/home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32: localization_data_pub/CMakeFiles/sendUartSTM32.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/datn_navbot/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32"
	cd /home/pi/datn_navbot/build/localization_data_pub && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sendUartSTM32.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
localization_data_pub/CMakeFiles/sendUartSTM32.dir/build: /home/pi/datn_navbot/devel/lib/localization_data_pub/sendUartSTM32

.PHONY : localization_data_pub/CMakeFiles/sendUartSTM32.dir/build

localization_data_pub/CMakeFiles/sendUartSTM32.dir/clean:
	cd /home/pi/datn_navbot/build/localization_data_pub && $(CMAKE_COMMAND) -P CMakeFiles/sendUartSTM32.dir/cmake_clean.cmake
.PHONY : localization_data_pub/CMakeFiles/sendUartSTM32.dir/clean

localization_data_pub/CMakeFiles/sendUartSTM32.dir/depend:
	cd /home/pi/datn_navbot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/datn_navbot/src /home/pi/datn_navbot/src/localization_data_pub /home/pi/datn_navbot/build /home/pi/datn_navbot/build/localization_data_pub /home/pi/datn_navbot/build/localization_data_pub/CMakeFiles/sendUartSTM32.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : localization_data_pub/CMakeFiles/sendUartSTM32.dir/depend

