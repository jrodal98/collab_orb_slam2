cmake_minimum_required(VERSION 2.4.6)

project(compression)


find_package(Boost REQUIRED COMPONENTS filesystem program_options)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Protobuf REQUIRED)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

SET(ROS_BUILD_TYPE Release)


rosbuild_init()
rosbuild_genmsg()


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -g -march=native -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -march=native -fopenmp")


# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)





set(LIBS 
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${Boost_LIBRARIES}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROTOBUF_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libCORB_SLAM2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/featureCompression2/install/lib/libLBFC2.so
)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../Thirdparty/featureCompression2/install/include
${Pangolin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}/../../../build/
)


### MONOCULAR ###

## SERVER EXAMPLE ##
rosbuild_add_executable(KittiServerMono
${PROJECT_SOURCE_DIR}/src/compression/KittiServerMono.cpp
)
target_link_libraries(KittiServerMono
${LIBS}
)

