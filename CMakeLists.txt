cmake_minimum_required(VERSION 3.5.1)
project(icp_registration VERSION 0.1)

# Make sure no in source builds
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "Error: Please create a separate directory for build files,
in-source builds are not allowed.")
endif()

# Find existing libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# Include directories 
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Link Directories 
link_directories(${EIGEN3_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Add Definitions 
add_definitions(${EIGEN3_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})
	
# For Ubuntu 16.04, to address vtk bug with PCL
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# Specify compiler flags
#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -std=c++17")

# Get system information (possibly for different platforms)
message("System: ${CMAKE_SYSTEM_NAME}, Complete System Name: ${CMAKE_SYSTEM} ")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR} ")
message("CMake binary directory: ${CMAKE_BINARY_DIR}")
message("CMake source directory: ${CMAKE_SOURCE_DIR}")
message("Eigen version: ${EIGEN3_VERSION}")
message("PCL version: ${PCL_VERSION}")

# Specify project output paths
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${CMAKE_BINARY_DIR}/lib)

# User Libraries
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)
add_library(registration STATIC include/registration.hpp src/registration.cpp)
add_library(KDTree STATIC include/KDTree.hpp src/KDTree.cpp)
add_library(data STATIC include/data.hpp src/data.cpp)
add_library(pointcloud_viewer STATIC include/pointcloud_viewer.hpp src/pointcloud_viewer.cpp)

# Executable
add_executable(main main.cpp)
target_link_libraries(main registration KDTree data pointcloud_viewer ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

add_custom_target(run 
	COMMAND main
	DEPENDS main
	WORKING_DIRECTORY ${CMAKE_BINARY_DIRECTORY}
	)
