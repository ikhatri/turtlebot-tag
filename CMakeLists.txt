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

# Some disgusting crap needed because OpenCV 3.3.1 has strange bugs
# I spent an hour debugging this shit, see: https://github.com/ros-perception/vision_opencv/issues/193 for details
find_package(OpenCV 3 REQUIRED)
if (${OpenCV_VERSION} MATCHES "3.3.1")
  foreach(__cvcomponent ${OpenCV_LIB_COMPONENTS})
    set (__original_cvcomponent ${__cvcomponent})
    if(NOT __cvcomponent MATCHES "^opencv_")
      set(__cvcomponent opencv_${__cvcomponent})
    endif()
    if (TARGET ${__cvcomponent})
      set_target_properties(${__cvcomponent} PROPERTIES
          MAP_IMPORTED_CONFIG_DEBUG ""
          MAP_IMPORTED_CONFIG_RELEASE ""
          MAP_IMPORTED_CONFIG_RELWITHDEBINFO ""
          MAP_IMPORTED_CONFIG_MINSIZEREL ""
      )
    endif()
  endforeach(__cvcomponent)
endif()

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
TARGET_LINK_LIBRARIES(turtlebot_tag ${libs} ${OpenCV_LIBS})
ROSBUILD_LINK_BOOST(turtlebot_tag system)