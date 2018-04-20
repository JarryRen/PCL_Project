file(GLOB SOURCE_PCL ./pcl_project/*cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(qt_pcl_project main.cpp ${SOURCE_PCL})
target_link_libraries(qt_pcl_project ${PCL_LIBRARIES} ${OpenCV_LIBS})