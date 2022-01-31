project(data_import)

add_library(data_import STATIC
    src/data_import.cpp
)
    
target_include_directories(data_import
    PUBLIC include/data_import
    PRIVATE src
)

target_link_libraries(data_import 
    PUBLIC Eigen3::Eigen
)
