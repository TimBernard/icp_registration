project(registration)

add_library(registration STATIC
  src/registration.cpp
)
    
target_include_directories(registration
  PUBLIC 
    include/registration
  PRIVATE 
    src
)

target_link_libraries(registration 
  PUBLIC 
    kd_tree 
    pointcloud_viewer 
    Eigen3::Eigen 
)
