cmake_minimum_required(VERSION 2.8.3)
project(geom_cast)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen REQUIRED)

# catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs tf
  DEPENDS OpenCV PCL Eigen Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

## Declare a cpp library
# add_library(geom_cast
#   src/${PROJECT_NAME}/geom_cast.cpp
# )

## Declare a cpp executable
add_executable(${PROJECT_NAME}_examples src/examples.cpp)
target_link_libraries(${PROJECT_NAME}_examples
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(geom_cast_node geom_cast_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(geom_cast_node
#   ${catkin_LIBRARIES}
# )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}_test utest/test.cpp)
endif()

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_geom_cast.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
