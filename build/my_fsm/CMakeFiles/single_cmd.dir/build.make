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
CMAKE_SOURCE_DIR = /home/flag/FLAG_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/flag/FLAG_ws/build

# Include any dependencies generated for this target.
include my_fsm/CMakeFiles/single_cmd.dir/depend.make

# Include the progress variables for this target.
include my_fsm/CMakeFiles/single_cmd.dir/progress.make

# Include the compile flags for this target's objects.
include my_fsm/CMakeFiles/single_cmd.dir/flags.make

my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o: my_fsm/CMakeFiles/single_cmd.dir/flags.make
my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o: /home/flag/FLAG_ws/src/my_fsm/src/single_cmd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/flag/FLAG_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o"
	cd /home/flag/FLAG_ws/build/my_fsm && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o -c /home/flag/FLAG_ws/src/my_fsm/src/single_cmd.cpp

my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/single_cmd.dir/src/single_cmd.cpp.i"
	cd /home/flag/FLAG_ws/build/my_fsm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/flag/FLAG_ws/src/my_fsm/src/single_cmd.cpp > CMakeFiles/single_cmd.dir/src/single_cmd.cpp.i

my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/single_cmd.dir/src/single_cmd.cpp.s"
	cd /home/flag/FLAG_ws/build/my_fsm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/flag/FLAG_ws/src/my_fsm/src/single_cmd.cpp -o CMakeFiles/single_cmd.dir/src/single_cmd.cpp.s

my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.requires:

.PHONY : my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.requires

my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.provides: my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.requires
	$(MAKE) -f my_fsm/CMakeFiles/single_cmd.dir/build.make my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.provides.build
.PHONY : my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.provides

my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.provides.build: my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o


# Object files for target single_cmd
single_cmd_OBJECTS = \
"CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o"

# External object files for target single_cmd
single_cmd_EXTERNAL_OBJECTS =

/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: my_fsm/CMakeFiles/single_cmd.dir/build.make
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/libroscpp.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/librosconsole.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/librostime.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /opt/ros/melodic/lib/libcpp_common.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd: my_fsm/CMakeFiles/single_cmd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/flag/FLAG_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd"
	cd /home/flag/FLAG_ws/build/my_fsm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/single_cmd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
my_fsm/CMakeFiles/single_cmd.dir/build: /home/flag/FLAG_ws/devel/lib/my_fsm/single_cmd

.PHONY : my_fsm/CMakeFiles/single_cmd.dir/build

my_fsm/CMakeFiles/single_cmd.dir/requires: my_fsm/CMakeFiles/single_cmd.dir/src/single_cmd.cpp.o.requires

.PHONY : my_fsm/CMakeFiles/single_cmd.dir/requires

my_fsm/CMakeFiles/single_cmd.dir/clean:
	cd /home/flag/FLAG_ws/build/my_fsm && $(CMAKE_COMMAND) -P CMakeFiles/single_cmd.dir/cmake_clean.cmake
.PHONY : my_fsm/CMakeFiles/single_cmd.dir/clean

my_fsm/CMakeFiles/single_cmd.dir/depend:
	cd /home/flag/FLAG_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/flag/FLAG_ws/src /home/flag/FLAG_ws/src/my_fsm /home/flag/FLAG_ws/build /home/flag/FLAG_ws/build/my_fsm /home/flag/FLAG_ws/build/my_fsm/CMakeFiles/single_cmd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : my_fsm/CMakeFiles/single_cmd.dir/depend
