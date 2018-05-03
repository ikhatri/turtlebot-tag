cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_compile_options(-std=c++11)

ROSBUILD_INIT()

ROSBUILD_ADD_BOOST_DIRECTORIES()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(turtlebot_sim src/assignment4.cpp)
TARGET_LINK_LIBRARIES(turtlebot_sim ${libs})
ROSBUILD_LINK_BOOST(turtlebot_sim system)

ROSBUILD_ADD_EXECUTABLE(assignment5 src/assignment5.cpp)
TARGET_LINK_LIBRARIES(assignment5 ${libs})
ROSBUILD_LINK_BOOST(assignment5 system)

ROSBUILD_ADD_EXECUTABLE(get_transform src/get_transform.cpp)
TARGET_LINK_LIBRARIES(get_transform ${libs})
ROSBUILD_LINK_BOOST(get_transform system)

ROSBUILD_ADD_EXECUTABLE(turtlebot_tag src/turtlebot_tag.cpp)
TARGET_LINK_LIBRARIES(turtlebot_tag ${libs})
ROSBUILD_LINK_BOOST(turtlebot_tag system)