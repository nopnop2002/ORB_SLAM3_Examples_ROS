# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/nop/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/nop/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3/build

# Utility rule file for rosbuild_clean-test-results.

# Include any custom commands dependencies for this target.
include CMakeFiles/rosbuild_clean-test-results.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/rosbuild_clean-test-results.dir/progress.make

CMakeFiles/rosbuild_clean-test-results:
	if ! rm -rf /home/nop/.ros/test_results/ORB_SLAM3; then echo "WARNING:\ failed\ to\ remove\ test-results\ directory"; fi

rosbuild_clean-test-results: CMakeFiles/rosbuild_clean-test-results
rosbuild_clean-test-results: CMakeFiles/rosbuild_clean-test-results.dir/build.make
.PHONY : rosbuild_clean-test-results

# Rule to build all files generated by this target.
CMakeFiles/rosbuild_clean-test-results.dir/build: rosbuild_clean-test-results
.PHONY : CMakeFiles/rosbuild_clean-test-results.dir/build

CMakeFiles/rosbuild_clean-test-results.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rosbuild_clean-test-results.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rosbuild_clean-test-results.dir/clean

CMakeFiles/rosbuild_clean-test-results.dir/depend:
	cd /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3 /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3 /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3/build /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3/build /home/nop/ORB_SLAM3/Examples/ROS/ORB_SLAM3/build/CMakeFiles/rosbuild_clean-test-results.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rosbuild_clean-test-results.dir/depend

