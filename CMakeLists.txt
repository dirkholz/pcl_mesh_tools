cmake_minimum_required (VERSION 2.8 FATAL_ERROR) 
project (mesh_tools) 

set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin) 
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) 

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_libraries(${PCL_LIBRARIES} boost_thread boost_filesystem boost_system pthread)

add_executable(pcl_mesh_convert src/pcl_mesh_convert.cpp)
add_executable(pcl_mesh_view src/pcl_mesh_view.cpp)
# add_executable(pcl_mesh_render_views src/pcl_mesh_render_views.cpp)
# add_executable(global_classification src/global_classification.cpp)

