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
CMAKE_COMMAND = /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake

# The command to remove a file.
RM = /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nxf75284/verisilicon_sw_isp/appshell

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nxf75284/verisilicon_sw_isp/appshell/build

# Utility rule file for copy_libs_cam_engine.

# Include any custom commands dependencies for this target.
include obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/compiler_depend.make

# Include the progress variables for this target.
include obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/progress.make

obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/nxf75284/verisilicon_sw_isp/appshell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "##Copy cam_engine headers to include"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/cam_engine/proprietories && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_directory /home/nxf75284/verisilicon_sw_isp/units/cam_engine/proprietories/include /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/cam_engine/proprietories && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy /home/nxf75284/verisilicon_sw_isp/units/cam_engine/proprietories/lib/libcam_engine.so /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/lib/libcam_engine.so

copy_libs_cam_engine: obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine
copy_libs_cam_engine: obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/build.make
.PHONY : copy_libs_cam_engine

# Rule to build all files generated by this target.
obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/build: copy_libs_cam_engine
.PHONY : obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/build

obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/clean:
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/cam_engine/proprietories && $(CMAKE_COMMAND) -P CMakeFiles/copy_libs_cam_engine.dir/cmake_clean.cmake
.PHONY : obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/clean

obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/depend:
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nxf75284/verisilicon_sw_isp/appshell /home/nxf75284/verisilicon_sw_isp/units/cam_engine/proprietories /home/nxf75284/verisilicon_sw_isp/appshell/build /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/cam_engine/proprietories /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : obj/units/obj/cam_engine/proprietories/CMakeFiles/copy_libs_cam_engine.dir/depend

