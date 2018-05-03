cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

ROSBUILD_INIT()

ROSBUILD_ADD_BOOST_DIRECTORIES()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(assignment4 src/assignment4.cpp)
TARGET_LINK_LIBRARIES(assignment4 ${libs})

ROSBUILD_LINK_BOOST(assignment4 system)
