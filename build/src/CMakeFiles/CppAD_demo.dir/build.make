# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hamster/mycode/mpc_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hamster/mycode/mpc_controller/build

# Include any dependencies generated for this target.
include src/CMakeFiles/CppAD_demo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/CppAD_demo.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/CppAD_demo.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/CppAD_demo.dir/flags.make

src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o: src/CMakeFiles/CppAD_demo.dir/flags.make
src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o: ../src/testcppad/CppAD_demo.cpp
src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o: src/CMakeFiles/CppAD_demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hamster/mycode/mpc_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o"
	cd /home/hamster/mycode/mpc_controller/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o -MF CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o.d -o CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o -c /home/hamster/mycode/mpc_controller/src/testcppad/CppAD_demo.cpp

src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.i"
	cd /home/hamster/mycode/mpc_controller/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hamster/mycode/mpc_controller/src/testcppad/CppAD_demo.cpp > CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.i

src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.s"
	cd /home/hamster/mycode/mpc_controller/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hamster/mycode/mpc_controller/src/testcppad/CppAD_demo.cpp -o CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.s

# Object files for target CppAD_demo
CppAD_demo_OBJECTS = \
"CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o"

# External object files for target CppAD_demo
CppAD_demo_EXTERNAL_OBJECTS =

../bin/CppAD_demo: src/CMakeFiles/CppAD_demo.dir/testcppad/CppAD_demo.cpp.o
../bin/CppAD_demo: src/CMakeFiles/CppAD_demo.dir/build.make
../bin/CppAD_demo: src/CMakeFiles/CppAD_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hamster/mycode/mpc_controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/CppAD_demo"
	cd /home/hamster/mycode/mpc_controller/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CppAD_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/CppAD_demo.dir/build: ../bin/CppAD_demo
.PHONY : src/CMakeFiles/CppAD_demo.dir/build

src/CMakeFiles/CppAD_demo.dir/clean:
	cd /home/hamster/mycode/mpc_controller/build/src && $(CMAKE_COMMAND) -P CMakeFiles/CppAD_demo.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/CppAD_demo.dir/clean

src/CMakeFiles/CppAD_demo.dir/depend:
	cd /home/hamster/mycode/mpc_controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hamster/mycode/mpc_controller /home/hamster/mycode/mpc_controller/src /home/hamster/mycode/mpc_controller/build /home/hamster/mycode/mpc_controller/build/src /home/hamster/mycode/mpc_controller/build/src/CMakeFiles/CppAD_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/CppAD_demo.dir/depend
