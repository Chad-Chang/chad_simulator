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
CMAKE_SOURCE_DIR = /home/chad/Downloads/eigen-3.4.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator

# Include any dependencies generated for this target.
include doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/depend.make

# Include the progress variables for this target.
include doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/progress.make

# Include the compile flags for this target's objects.
include doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/flags.make

doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.o: doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/flags.make
doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.o: /home/chad/Downloads/eigen-3.4.0/doc/examples/tut_arithmetic_dot_cross.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.o"
	cd /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.o -c /home/chad/Downloads/eigen-3.4.0/doc/examples/tut_arithmetic_dot_cross.cpp

doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.i"
	cd /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chad/Downloads/eigen-3.4.0/doc/examples/tut_arithmetic_dot_cross.cpp > CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.i

doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.s"
	cd /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chad/Downloads/eigen-3.4.0/doc/examples/tut_arithmetic_dot_cross.cpp -o CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.s

# Object files for target tut_arithmetic_dot_cross
tut_arithmetic_dot_cross_OBJECTS = \
"CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.o"

# External object files for target tut_arithmetic_dot_cross
tut_arithmetic_dot_cross_EXTERNAL_OBJECTS =

doc/examples/tut_arithmetic_dot_cross: doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/tut_arithmetic_dot_cross.cpp.o
doc/examples/tut_arithmetic_dot_cross: doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/build.make
doc/examples/tut_arithmetic_dot_cross: doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable tut_arithmetic_dot_cross"
	cd /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tut_arithmetic_dot_cross.dir/link.txt --verbose=$(VERBOSE)
	cd /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples && ./tut_arithmetic_dot_cross >/home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples/tut_arithmetic_dot_cross.out

# Rule to build all files generated by this target.
doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/build: doc/examples/tut_arithmetic_dot_cross

.PHONY : doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/build

doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/clean:
	cd /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples && $(CMAKE_COMMAND) -P CMakeFiles/tut_arithmetic_dot_cross.dir/cmake_clean.cmake
.PHONY : doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/clean

doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/depend:
	cd /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chad/Downloads/eigen-3.4.0 /home/chad/Downloads/eigen-3.4.0/doc/examples /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples /home/chad/Documents/mujoco-2.2.1/myProject/chad_simulator/doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : doc/examples/CMakeFiles/tut_arithmetic_dot_cross.dir/depend

