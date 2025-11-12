# Test script to verify PreBuild.cmake works
cmake_minimum_required(VERSION 3.16)

set(CMAKE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")

# Load and test the pre-build script
include("${CMAKE_CURRENT_LIST_DIR}/cmake/PreBuild.cmake")

# Execute pre-build automation
ardupilot_prebuild()

message(STATUS "")
message(STATUS "==============================================")
message(STATUS "Pre-Build Test Completed Successfully!")
message(STATUS "==============================================")
