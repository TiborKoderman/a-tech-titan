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
CMAKE_SOURCE_DIR = /home/tibor/repos/a-tech-titan/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tibor/repos/a-tech-titan/build

# Utility rule file for exercise1_geneus.

# Include the progress variables for this target.
include exercise1/CMakeFiles/exercise1_geneus.dir/progress.make

exercise1_geneus: exercise1/CMakeFiles/exercise1_geneus.dir/build.make

.PHONY : exercise1_geneus

# Rule to build all files generated by this target.
exercise1/CMakeFiles/exercise1_geneus.dir/build: exercise1_geneus

.PHONY : exercise1/CMakeFiles/exercise1_geneus.dir/build

exercise1/CMakeFiles/exercise1_geneus.dir/clean:
	cd /home/tibor/repos/a-tech-titan/build/exercise1 && $(CMAKE_COMMAND) -P CMakeFiles/exercise1_geneus.dir/cmake_clean.cmake
.PHONY : exercise1/CMakeFiles/exercise1_geneus.dir/clean

exercise1/CMakeFiles/exercise1_geneus.dir/depend:
	cd /home/tibor/repos/a-tech-titan/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tibor/repos/a-tech-titan/src /home/tibor/repos/a-tech-titan/src/exercise1 /home/tibor/repos/a-tech-titan/build /home/tibor/repos/a-tech-titan/build/exercise1 /home/tibor/repos/a-tech-titan/build/exercise1/CMakeFiles/exercise1_geneus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : exercise1/CMakeFiles/exercise1_geneus.dir/depend

