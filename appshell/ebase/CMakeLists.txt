
cmake_minimum_required(VERSION 2.6)

# define module name here
set (module appshell_ebase)

# we want to compile all .c files as default
file(GLOB libsources source/*.c)

# set public headers, these get installed
file(GLOB pub_headers include/*.h)

# define include paths
include_directories(
    include
    include_priv
    ${LIB_ROOT}/${CMAKE_BUILD_TYPE}/include
    )

# module specific defines
add_definitions(-D_FILE_OFFSET_BITS=64 -DHAS_STDINT_H -Wno-error=unused-function -Wno-array-bounds)
add_definitions(-fPIC)

# add lib to build env
add_library(${module}_shared SHARED ${libsources})

SET_TARGET_PROPERTIES(${module}_shared PROPERTIES OUTPUT_NAME     ${module})
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES LINK_FLAGS      -shared)
SET_TARGET_PROPERTIES(${module}_shared PROPERTIES FRAMEWORK       TRUE PUBLIC_HEADER "${pub_headers}")

# define lib dependencies
target_link_libraries(${module}_shared
#                      rt
#                      pthread
        ${LIB_RT_NAME}
        ${LIB_PTHREAD_NAME}
        )

# define stuff to install
install(TARGETS ${module}_shared
        PUBLIC_HEADER   DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${module}
        ARCHIVE         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${module}
        LIBRARY         DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${module}
        )

# create common targets for this module
include(${CMAKE_HOME_DIRECTORY}/targets.cmake)

