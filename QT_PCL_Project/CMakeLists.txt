cmake_minimum_required(VERSION 3.6)

project(qt_pcl_project)

set(CMAKE_BUILD_TYPE "Debug")

#PCL
find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#OpenCV
find_package( OpenCV REQUIRED )

#Qt
find_package( Qt5 REQUIRED Widgets Core )

#gp
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
link_directories(${PROJECT_SOURCE_DIR}/lib)
