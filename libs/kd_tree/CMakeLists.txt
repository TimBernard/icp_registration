project(kd_tree)

add_library(kd_tree STATIC
  src/kd_tree.cpp
)
    
target_include_directories(kd_tree
  PUBLIC 
    include/kd_tree
  PRIVATE 
    src
)

target_link_libraries(kd_tree 
  PUBLIC 
    Eigen3::Eigen
)
