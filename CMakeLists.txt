cmake_minimum_required(VERSION 2.8.3)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

if(NOT ROS_BUILD_TYPE)
  set(ROS_BUILD_TYPE Release)
endif()

message("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Gflags REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${GFLAGS_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
)

rosbuild_add_executable(${PROJECT_NAME}
src/bag_to_image.cpp
)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} filesystem system)

target_link_libraries(${PROJECT_NAME}
${GFLAGS_LIBRARY}
${OpenCV_LIBRARIES}
)

