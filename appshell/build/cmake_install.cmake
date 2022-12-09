# Install script for directory: /home/nxf75284/verisilicon_sw_isp/appshell

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nxf75284/verisilicon_sw_isp/appshell/build/dist")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/nxf75284/toolchain_GCC_2.35/5.15-kirkstone/sysroots/x86_64-pokysdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/appshell_oslayer/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/appshell_ibd/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/jsoncpp/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/appshell_ebase/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/yuv/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/display/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/units/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/mediacontrol/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/appshell_hal/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/v4l_drm_test/cmake_install.cmake")
  include("/home/nxf75284/verisilicon_sw_isp/appshell/build/obj/vvext/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/nxf75284/verisilicon_sw_isp/appshell/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
