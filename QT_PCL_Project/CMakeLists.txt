project(qt_pcl_project)

set(CMAKE_BUILD_TYPE "Debug")

#PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#OpenCV
FIND_PACKAGE( OpenCV REQUIRED )

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/pcl_project/)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
