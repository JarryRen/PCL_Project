file(GLOB SOURCE_ALL ./*cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(PCL_Project ${SOURCE_ALL})
target_link_libraries(PCL_Project ${PCL_LIBRARIES} ${OpenCV_LIBS})
