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

# Include any dependencies generated for this target.
include obj/units/obj/isi/CMakeFiles/isi_shared.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include obj/units/obj/isi/CMakeFiles/isi_shared.dir/compiler_depend.make

# Include the progress variables for this target.
include obj/units/obj/isi/CMakeFiles/isi_shared.dir/progress.make

# Include the compile flags for this target's objects.
include obj/units/obj/isi/CMakeFiles/isi_shared.dir/flags.make

obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.o: obj/units/obj/isi/CMakeFiles/isi_shared.dir/flags.make
obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.o: /home/nxf75284/verisilicon_sw_isp/units/isi/source/isi.c
obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.o: obj/units/obj/isi/CMakeFiles/isi_shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nxf75284/verisilicon_sw_isp/appshell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.o"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && ccache /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc   -mcpu=cortex-a53 -march=armv8-a+crc+crypto -fstack-protector-strong  -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.o -MF CMakeFiles/isi_shared.dir/source/isi.c.o.d -o CMakeFiles/isi_shared.dir/source/isi.c.o -c /home/nxf75284/verisilicon_sw_isp/units/isi/source/isi.c

obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/isi_shared.dir/source/isi.c.i"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc   -mcpu=cortex-a53 -march=armv8-a+crc+crypto -fstack-protector-strong  -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nxf75284/verisilicon_sw_isp/units/isi/source/isi.c > CMakeFiles/isi_shared.dir/source/isi.c.i

obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/isi_shared.dir/source/isi.c.s"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc   -mcpu=cortex-a53 -march=armv8-a+crc+crypto -fstack-protector-strong  -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nxf75284/verisilicon_sw_isp/units/isi/source/isi.c -o CMakeFiles/isi_shared.dir/source/isi.c.s

obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.o: obj/units/obj/isi/CMakeFiles/isi_shared.dir/flags.make
obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.o: /home/nxf75284/verisilicon_sw_isp/units/isi/source/isisup.c
obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.o: obj/units/obj/isi/CMakeFiles/isi_shared.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nxf75284/verisilicon_sw_isp/appshell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.o"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && ccache /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc   -mcpu=cortex-a53 -march=armv8-a+crc+crypto -fstack-protector-strong  -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.o -MF CMakeFiles/isi_shared.dir/source/isisup.c.o.d -o CMakeFiles/isi_shared.dir/source/isisup.c.o -c /home/nxf75284/verisilicon_sw_isp/units/isi/source/isisup.c

obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/isi_shared.dir/source/isisup.c.i"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc   -mcpu=cortex-a53 -march=armv8-a+crc+crypto -fstack-protector-strong  -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/nxf75284/verisilicon_sw_isp/units/isi/source/isisup.c > CMakeFiles/isi_shared.dir/source/isisup.c.i

obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/isi_shared.dir/source/isisup.c.s"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && /home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-gcc   -mcpu=cortex-a53 -march=armv8-a+crc+crypto -fstack-protector-strong  -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security -Werror=format-security --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux --sysroot=/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/cortexa53-crypto-poky-linux $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/nxf75284/verisilicon_sw_isp/units/isi/source/isisup.c -o CMakeFiles/isi_shared.dir/source/isisup.c.s

# Object files for target isi_shared
isi_shared_OBJECTS = \
"CMakeFiles/isi_shared.dir/source/isi.c.o" \
"CMakeFiles/isi_shared.dir/source/isisup.c.o"

# External object files for target isi_shared
isi_shared_EXTERNAL_OBJECTS =

generated/release/lib/libisi.so: obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isi.c.o
generated/release/lib/libisi.so: obj/units/obj/isi/CMakeFiles/isi_shared.dir/source/isisup.c.o
generated/release/lib/libisi.so: obj/units/obj/isi/CMakeFiles/isi_shared.dir/build.make
generated/release/lib/libisi.so: obj/units/obj/isi/CMakeFiles/isi_shared.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nxf75284/verisilicon_sw_isp/appshell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C shared library ../../../../generated/release/lib/libisi.so"
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/isi_shared.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
obj/units/obj/isi/CMakeFiles/isi_shared.dir/build: generated/release/lib/libisi.so
.PHONY : obj/units/obj/isi/CMakeFiles/isi_shared.dir/build

obj/units/obj/isi/CMakeFiles/isi_shared.dir/clean:
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi && $(CMAKE_COMMAND) -P CMakeFiles/isi_shared.dir/cmake_clean.cmake
.PHONY : obj/units/obj/isi/CMakeFiles/isi_shared.dir/clean

obj/units/obj/isi/CMakeFiles/isi_shared.dir/depend:
	cd /home/nxf75284/verisilicon_sw_isp/appshell/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nxf75284/verisilicon_sw_isp/appshell /home/nxf75284/verisilicon_sw_isp/units/isi /home/nxf75284/verisilicon_sw_isp/appshell/build /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi /home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/obj/isi/CMakeFiles/isi_shared.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : obj/units/obj/isi/CMakeFiles/isi_shared.dir/depend

