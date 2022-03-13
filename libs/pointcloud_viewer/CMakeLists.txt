project(pointcloud_viewer)

add_library(pointcloud_viewer STATIC
  src/pointcloud_viewer.cpp
)
    
target_include_directories(pointcloud_viewer
  PUBLIC 
    include/pointcloud_viewer
  PRIVATE 
    src 
)

target_link_libraries(pointcloud_viewer
  PUBLIC 
    ${PCL_LIBRARIES} 
    Eigen3::Eigen 
    Threads::Threads
)
