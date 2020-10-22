cmake_minimum_required(VERSION 3.2)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
)

file(GLOB CV_BRIDGE_CPP_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/vision_opencv/cv_bridge/src/cv_bridge.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/vision_opencv/cv_bridge/src/rgb_colors.cpp")
#file(GLOB CV_BRIDGE_HPP_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/cv_bridge/src/*.hpp")

#include_directories(../ros/ros_comm/tools/rosconsole/include)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/vision_opencv/cv_bridge/include" "${CMAKE_CURRENT_SOURCE_DIR}/vision_opencv/cv_bridge/src" ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_library(cv_bridge ${CV_BRIDGE_CPP_SRCS} ) #${CV_BRIDGE_CPP_SRCS})
target_include_directories(cv_bridge PUBLIC ${OpenCV_INCLUDE_DIR})
target_link_libraries(cv_bridge PUBLIC ${Boost_LIBS} ${OpenCV_LIBS})

#add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/cv_bridge/src")


