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

# Utility rule file for ebase_copy_header.

# Include any custom commands dependencies for this target.
include obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/compiler_depend.make

# Include the progress variables for this target.
include obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/progress.make

obj/units/obj/ebase/CMakeFiles/ebase_copy_header:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/nxf75284/verisilicon_sw_isp/appshell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Copying Headers of ebase"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/builtins.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/dct_assert.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/ext_types.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/hashtable.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/linux_compat.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/list.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/mainpage.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/queue.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/return_types.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/slist.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/sort.frag.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/trace.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/cmake -E copy_if_different /home/nxf75284/verisilicon_sw_isp/units/ebase/include/types.h /home/nxf75284/verisilicon_sw_isp/appshell/build/generated/release/include/ebase/

ebase_copy_header: obj/units/obj/ebase/CMakeFiles/ebase_copy_header
ebase_copy_header: obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/build.make
.PHONY : ebase_copy_header

# Rule to build all files generated by this target.
obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/build: ebase_copy_header
.PHONY : obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/build

obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/clean:
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase && $(CMAKE_COMMAND) -P CMakeFiles/ebase_copy_header.dir/cmake_clean.cmake
.PHONY : obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/clean

obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/depend:
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nxf75284/verisilicon_sw_isp/appshell /home/nxf75284/verisilicon_sw_isp/units/ebase /home/nxf75284/verisilicon_sw_isp/appshell/build /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : obj/units/obj/ebase/CMakeFiles/ebase_copy_header.dir/depend
