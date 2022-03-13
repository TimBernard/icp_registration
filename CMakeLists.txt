cmake_minimum_required(VERSION 3.16)
project(icp_registration VERSION 0.4)

# Make sure no in source builds
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "Error: Please create a separate directory for build files,
in-source builds are not allowed.")
endif()

set(CMAKE_CXX_STANDARD 17)

# Find third-party libraries
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Get system information 
message("System: ${CMAKE_SYSTEM_NAME}, Complete System Name: ${CMAKE_SYSTEM} ")
message("CMake binary directory: ${CMAKE_BINARY_DIR}")
message("CMake source directory: ${CMAKE_SOURCE_DIR}")
message("Eigen version: ${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION}")
message("PCL version: ${PCL_VERSION}")

# Specify project output paths
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# Add all in-house libraries 
add_subdirectory(libs)

# Executable
add_executable(main src/main.cpp)
target_link_libraries(main 
  PRIVATE 
    registration 
    data_import 
)

add_custom_target(run 
  COMMAND main
  DEPENDS main
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
