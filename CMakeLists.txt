cmake_minimum_required(VERSION 2.8)
project(calibu_tracker)

# find packages

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
list(APPEND REQUIRED_INCLUDE_DIRS ${CMAKE_SOURCE_DIR})

find_package(Calibu REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${Calibu_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${Calibu_LIBRARIES})

find_package(Eigen3 REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${EIGEN3_LIBRARIES})

find_package(HAL REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${HAL_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${HAL_LIBRARIES})

find_package(OpenCV REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${OpenCV_LIBRARIES})

find_package(Sophus REQUIRED)
list(APPEND REQUIRED_INCLUDE_DIRS ${Sophus_INCLUDE_DIRS})
list(APPEND REQUIRED_LIBRARIES ${Sophus_LIBRARIES})

# set compiler flags

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# configure files

configure_file(image0.png image0.png COPYONLY)
configure_file(image1.png image1.png COPYONLY)
configure_file(image2.png image2.png COPYONLY)
configure_file(cameras.xml cameras.xml COPYONLY)

# build executable

add_executable(calibu_tracker main.cpp target_tracker.cpp)
target_link_libraries(calibu_tracker ${REQUIRED_LIBRARIES})
target_include_directories(calibu_tracker PUBLIC ${REQUIRED_INCLUDE_DIRS})