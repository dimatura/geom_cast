cmake_minimum_required(VERSION 2.8.3)

find_package(OpenCV)
find_package(PCL)

if (PCL_FOUND)
    add_definitions(-DGEOMCAST_USE_PCL)
endif(PCL_FOUND)

if (OpenCV_FOUND)
    add_definitions(-DGEOMCAST_USE_OPENCV)
endif(OpenCV_FOUND)
