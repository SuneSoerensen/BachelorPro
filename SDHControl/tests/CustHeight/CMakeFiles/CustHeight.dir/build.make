# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/sune/BachelorPro/SDHControl/tests/CustHeight

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sune/BachelorPro/SDHControl/tests/CustHeight

# Include any dependencies generated for this target.
include CMakeFiles/CustHeight.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CustHeight.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CustHeight.dir/flags.make

CMakeFiles/CustHeight.dir/CustHeight.cpp.o: CMakeFiles/CustHeight.dir/flags.make
CMakeFiles/CustHeight.dir/CustHeight.cpp.o: CustHeight.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sune/BachelorPro/SDHControl/tests/CustHeight/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CustHeight.dir/CustHeight.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CustHeight.dir/CustHeight.cpp.o -c /home/sune/BachelorPro/SDHControl/tests/CustHeight/CustHeight.cpp

CMakeFiles/CustHeight.dir/CustHeight.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CustHeight.dir/CustHeight.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sune/BachelorPro/SDHControl/tests/CustHeight/CustHeight.cpp > CMakeFiles/CustHeight.dir/CustHeight.cpp.i

CMakeFiles/CustHeight.dir/CustHeight.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CustHeight.dir/CustHeight.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sune/BachelorPro/SDHControl/tests/CustHeight/CustHeight.cpp -o CMakeFiles/CustHeight.dir/CustHeight.cpp.s

CMakeFiles/CustHeight.dir/CustHeight.cpp.o.requires:

.PHONY : CMakeFiles/CustHeight.dir/CustHeight.cpp.o.requires

CMakeFiles/CustHeight.dir/CustHeight.cpp.o.provides: CMakeFiles/CustHeight.dir/CustHeight.cpp.o.requires
	$(MAKE) -f CMakeFiles/CustHeight.dir/build.make CMakeFiles/CustHeight.dir/CustHeight.cpp.o.provides.build
.PHONY : CMakeFiles/CustHeight.dir/CustHeight.cpp.o.provides

CMakeFiles/CustHeight.dir/CustHeight.cpp.o.provides.build: CMakeFiles/CustHeight.dir/CustHeight.cpp.o


CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o: CMakeFiles/CustHeight.dir/flags.make
CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o: /home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sune/BachelorPro/SDHControl/tests/CustHeight/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o -c /home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp

CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp > CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.i

CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp -o CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.s

CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.requires:

.PHONY : CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.requires

CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.provides: CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.requires
	$(MAKE) -f CMakeFiles/CustHeight.dir/build.make CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.provides.build
.PHONY : CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.provides

CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.provides.build: CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o


# Object files for target CustHeight
CustHeight_OBJECTS = \
"CMakeFiles/CustHeight.dir/CustHeight.cpp.o" \
"CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o"

# External object files for target CustHeight
CustHeight_EXTERNAL_OBJECTS =

bin/CustHeight: CMakeFiles/CustHeight.dir/CustHeight.cpp.o
bin/CustHeight: CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o
bin/CustHeight: CMakeFiles/CustHeight.dir/build.make
bin/CustHeight: libs/libSDHControl.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_algorithms.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_pathplanners.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_pathoptimization.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_simulation.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_opengl.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_assembly.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_task.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_calibration.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_csg.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_control.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_proximitystrategies.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/libyaobi.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/libpqp.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/libfcl.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libGL.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libxerces-c.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_assimp.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_regex.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libpthread.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_qhull.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_csgjs.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_unzip.a
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libz.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libdl.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_camera.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_can.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_dockwelder.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_pcube.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_serialport.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_tactile.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_motomanIA20.a
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_sdh.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_universalrobots.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_schunkpg70.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_netft.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_robolabFT.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_robotiq.so
bin/CustHeight: /usr/local/lib/libSDHLibrary-CPP.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libdc1394.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_algorithms.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_pathplanners.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_pathoptimization.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_simulation.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_opengl.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_assembly.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_task.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_calibration.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_csg.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_control.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_proximitystrategies.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/libyaobi.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/libpqp.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/libfcl.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libGLU.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libGL.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libxerces-c.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_assimp.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_regex.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libpthread.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_test_exec_monitor.a
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libboost_unit_test_framework.so
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_qhull.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_csgjs.a
bin/CustHeight: /home/sune/RobWork/RobWork/libs/release/librw_unzip.a
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libz.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libdl.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_camera.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_can.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_dockwelder.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_pcube.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_serialport.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_tactile.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_motomanIA20.a
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_sdh.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_universalrobots.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_schunkpg70.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_netft.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_robolabFT.so
bin/CustHeight: /home/sune/RobWork/RobWorkHardware/libs/relwithdebinfo/librwhw_robotiq.so
bin/CustHeight: /usr/local/lib/libSDHLibrary-CPP.so
bin/CustHeight: /usr/lib/x86_64-linux-gnu/libdc1394.so
bin/CustHeight: CMakeFiles/CustHeight.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sune/BachelorPro/SDHControl/tests/CustHeight/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable bin/CustHeight"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CustHeight.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CustHeight.dir/build: bin/CustHeight

.PHONY : CMakeFiles/CustHeight.dir/build

CMakeFiles/CustHeight.dir/requires: CMakeFiles/CustHeight.dir/CustHeight.cpp.o.requires
CMakeFiles/CustHeight.dir/requires: CMakeFiles/CustHeight.dir/home/sune/BachelorPro/SDHControl/src/sdhoptions.cpp.o.requires

.PHONY : CMakeFiles/CustHeight.dir/requires

CMakeFiles/CustHeight.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CustHeight.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CustHeight.dir/clean

CMakeFiles/CustHeight.dir/depend:
	cd /home/sune/BachelorPro/SDHControl/tests/CustHeight && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sune/BachelorPro/SDHControl/tests/CustHeight /home/sune/BachelorPro/SDHControl/tests/CustHeight /home/sune/BachelorPro/SDHControl/tests/CustHeight /home/sune/BachelorPro/SDHControl/tests/CustHeight /home/sune/BachelorPro/SDHControl/tests/CustHeight/CMakeFiles/CustHeight.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CustHeight.dir/depend
