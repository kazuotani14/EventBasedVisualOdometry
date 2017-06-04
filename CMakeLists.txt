cmake_minimum_required(VERSION 2.8.3)
project(evo)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dvs_msgs
  geometry_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(emvs_node src/cpp/emvs_node.cpp
			   			 src/cpp/keyframe_dsi.cpp
						 src/cpp/matlab_utils.cpp)
add_executable(image_converter src/cpp/image_converter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(emvs_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBS})
