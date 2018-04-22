#pcl
file(GLOB SOURCE_PCL ./pcl_project/*.cpp)

#qt
file(GLOB QT_SRC_FILE ./qt_project/*.cpp)
file(GLOB QT_UI_FILE ${PROJECT_SOURCE_DIR}/forms/*.ui)
file(GLOB QT_HEADER_FILE ${PROJECT_SOURCE_DIR}/include/qt_project/*.h)
file(GLOB QT_RCC_FILE ${PROJECT_SOURCE_DIR}/resource/*.qrc)

#利用qt5_wrap_ui可以把.ui文件包裹成一个ui_*.h文件，这里定义到${ui_FILES}变量里，在可执行文件中需要加入目标文件中, 利用qt5_add_resources（）命令，添加生成的qrc_*.cpp和其他源文件以及ui_*.h到target内
qt5_wrap_cpp(QT_HEADER_MOC ${QT_HEADER_FILE})
qt5_wrap_ui(QT_UI_MOC ${QT_UI_FILE})
qt5_add_resources(QT_RCC_MOC ${QT_RCC_FILE})

add_library(QT_LIB SHARED
    ${QT_SRC_FILE}
    ${QT_HEADER_MOC}
    ${QT_UI_MOC}
    ${QT_RCC_MOC}
    )
target_link_libraries(QT_LIB
    Qt5::Widgets)

#main project
add_executable(${PROJECT_NAME}
    main.cpp
    ${SOURCE_PCL}
    ${QT_SRC_FILE})

target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    QT_LIB)

#build path
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
