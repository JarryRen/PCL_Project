file(GLOB SOURCE_ALL ./*cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(scene_project ${SOURCE_ALL})
target_link_libraries(scene_project ${PCL_LIBRARIES} ${OpenCV_LIBS})