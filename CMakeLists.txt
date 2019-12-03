cmake_minimum_required(VERSION 2.8.3)
project(inter_iit_uav_fleet)

find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_executable(detector src/detect.cpp)
target_link_libraries(detector ${OpenCV_LIBRARIES})

cs_add_executable(planner src/planner.cpp)

cs_install()
cs_export()
