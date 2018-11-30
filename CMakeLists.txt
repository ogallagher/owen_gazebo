cmake_minimum_required(VERSION 2.8.3)
project(owen_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
## OWEN: $(pkg-config --cflags --libs sdl2) es necesario para SDL2
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	pioneer3at
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES owen_gazebo
#  CATKIN_DEPENDS pioneer3at
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
	owen_gazebo_client 
	src/owen_gazebo_client.cpp 
	src/laser.cpp 
	src/robot.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	owen_gazebo_client 
	${catkin_LIBRARIES}
)

add_executable(
	owen_explorer_grapher
	src/laser.cpp
	src/owen_explorer_grapher.cpp
	src/gnt_graph.cpp
	src/drawer.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
	owen_explorer_grapher
	${SDL2_LIBRARIES}
	${catkin_LIBRARIES}
)
