cmake_minimum_required(VERSION 3.9) 
project(AglTrackerPlugin)

# Add AGL Tracker plugin
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/plugin")

add_library(agl_tracker SHARED 
	${CMAKE_SOURCE_DIR}/plugin/agl_tracker.cc
    )
    
target_link_libraries(agl_tracker
	${GAZEBO_LIBRARIES}
	)