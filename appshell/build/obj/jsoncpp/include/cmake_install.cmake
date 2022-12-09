# Install script for directory: /home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include

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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/json" TYPE FILE FILES
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/allocator.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/assertions.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/autolink.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/config.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/features.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/forwards.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/json.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/reader.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/value.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/version.h"
    "/home/nxf75284/verisilicon_sw_isp/utils3rd/3rd/jsoncpp/include/json/writer.h"
    )
endif()

