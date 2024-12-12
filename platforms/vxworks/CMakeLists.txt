# CMakeLists.txt - px4_main build configuration
# Copyright (c) 2016, 2020 Wind River Systems, Inc. All Rights Reserved.
#
# This simple cmake project exercises all relevant build options
# (C, C++ and Assembler; Library, Sharedlib and Executable; Testcases).
# The associated build.sh script shows sample command-lines for 
# building in various environments. 
#
# modification history
# --------------------
# 13mar20,mze  modified to remove include_directories(src) (V7BC-2125)
# 18oct16,mob  written

# The syntax of this basic example works fine with cmake-2.8.7
# - cmake-2.8.9 or later is required for VxWorks Sharedlibs with Diab
# - cmake-3.3.0 or later is required for CROSSCOMPILING_EMULATOR to work
# - cmake-3.6.2 is recommended and has received most testing.
cmake_minimum_required(VERSION 2.8.7)

project(px4_main C CXX ASM)


# PX4-Autopilot
set(PX4_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" CACHE FILEPATH "PX4 source directory" FORCE)
set(PX4_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE FILEPATH "PX4 binary directory" FORCE)
message(STATUS "PX4_SOURCE_DIR: ${PX4_SOURCE_DIR}")
message(STATUS "PX4_BINARY_DIR: ${PX4_BINARY_DIR}")
list(APPEND CMAKE_MODULE_PATH ${PX4_SOURCE_DIR}/cmake)

include_directories(src/px4/lib)
include_directories(src/px4/lib/matrix)
include_directories(src/px4/platforms/common/include)
include_directories(src/px4/platforms/common/include/px4_platform_common)
include_directories(src/px4/platforms/posix/include)
include_directories(src/px4/platforms/posix/src/px4/common)
include_directories(src/px4/px4_vxworks_default)
include_directories(src/px4/src)

add_compile_definitions(__PX4_POSIX)
add_compile_definitions(__PX4_VXWORKS)

set(PX4_SHELL_COMMAND_PREFIX "px4-")
add_definitions("-DPX4_SHELL_COMMAND_PREFIX=\"${PX4_SHELL_COMMAND_PREFIX}\"")
# End OF PX4-Autopilot

#add_library(hellostatic STATIC src/static/hello.c src/static/hello.h)
#add_library(helloshared        src/shared/hello.c src/shared/hello.h)
## Defining a Version for the sharedlib is optional.
#set_target_properties(helloshared PROPERTIES SOVERSION 1)

add_executable(px4_main
	#src/helloworld.c
	src/px4/main.cpp
	#src/cpp/list.cpp
	#src/asm/addone.S
)
set_target_properties(px4_main PROPERTIES HAS_CXX TRUE)
#target_link_libraries(px4_main hellostatic helloshared)

## For shared libraries only, edit this to locate libc.so.1 on the target.
## By default, the toolchain finds sharedlibs on the host, through a debugger 
## hostfs connection, by compiling host path info into the executable.
#set(CMAKE_INSTALL_RPATH "/romfs/bin;/hostfs/ws/romfs/Contents/bin")

###################################################
# Testcases can be run with "make test" or "ctest".
###################################################
include(CTest)
add_test(NAME px4_main COMMAND px4_main)
set_property(TEST px4_main
    PROPERTY PASS_REGULAR_EXPRESSION "assembler: 1\\+1=2")

if(BUILD_TESTING)
  # Additional testcases are only built on request (default on):
  # A 2nd executable will link with the "C" Linker rather than C++.
  # It will validate that the test infrastructure properly reports failing tests.    
  #add_executable(failtest src/failtest.c)
  #add_test(NAME failtest COMMAND failtest)
  #set_property(TEST failtest PROPERTY WILL_FAIL TRUE)

  # Finally, validate that we can pass arguments, even with spaces
  add_test(NAME argtest COMMAND px4_main 3 "hello argtest" "hello argtest")
  set_property(TEST argtest
    PROPERTY PASS_REGULAR_EXPRESSION "argv.3.=.?hello argtest")
endif()