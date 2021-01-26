cmake_minimum_required(VERSION 3.10) 
# AGL Tracking Plugin
project(AglTrackerPlugin)

# Use uORB to receive vehicle status
set(PX4_DIR "${CMAKE_SOURCE_DIR}/../PX4/PX4-Autopilot")

find_package(gazebo REQUIRED) # Create a Gazebo plugin

include_directories(${GAZEBO_INCLUDE_DIRS} 
			   $ENV{GAZEBO_HOME}/gazebo/msgs
			   plugin/
			   ${PX4_DIR}/src/modules
			   ${PX4_DIR}/build/px4_sitl_default)

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/plugin")

add_library(agl_tracker SHARED 
	plugin/agl_tracker.cc)
    
target_link_libraries(agl_tracker
	${GAZEBO_LIBRARIES}	)
