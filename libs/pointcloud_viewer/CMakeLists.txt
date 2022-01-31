project(pointcloud_viewer)

add_library(pointcloud_viewer STATIC
    src/pointcloud_viewer.cpp
)
    
target_include_directories(pointcloud_viewer
    PUBLIC include/pointcloud_viewer
    PRIVATE src ${PCL_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS}
)

target_link_libraries(pointcloud_viewer
    PUBLIC Eigen3::Eigen ${PCL_LIBRARIES} Threads::Threads
)
