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
  pcl_ros
  pcl_conversions
  pcl_msgs
)

find_package(PCL REQUIRED)
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
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs pcl_ros
  DEPENDS PCL
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(emvs_node src/cpp/emvs_node.cpp
			   			 src/cpp/keyframe_dsi.cpp
						 src/cpp/filters.cpp
           src/cpp/utilities.cpp)
add_executable(test_image_converter src/cpp/test_image_converter.cpp)
add_executable(test_pcl src/cpp/test_pcl.cpp)
add_executable(test_opencv src/cpp/test_opencv.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(emvs_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(test_image_converter ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(test_pcl ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(test_opencv ${catkin_LIBRARIES} ${OpenCV_LIBS})
