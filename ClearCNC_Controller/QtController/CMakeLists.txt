cmake_minimum_required(VERSION 3.21)
project(ClearCNC_Controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS OpenGL OpenGLWidgets Widgets SerialPort Network Svg)

qt_add_executable(ClearCNC_Controller WIN32
    src/main.cpp
    src/MainWindow.h
    src/MainWindow.cpp
    src/PositionView3D.h
    src/PositionView3D.cpp
    src/GCodeProgramKinematics.h
    src/GCodeProgramKinematics.cpp
    resources/app_resources.qrc
)

target_link_libraries(ClearCNC_Controller PRIVATE
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    Qt6::Widgets
    Qt6::Svg
    Qt6::SerialPort
    Qt6::Network
)

# Windows: embed the .ico via a real .rc (Explorer + pinned taskbar; RC_ICORS alone is flaky on MSVC).
if (WIN32)
    set(_win_rc "${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/clearcnc_app_icon.rc")
    if (EXISTS "${_win_rc}")
        target_sources(ClearCNC_Controller PRIVATE "${_win_rc}")
        set_source_files_properties("${_win_rc}" PROPERTIES
            OBJECT_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/clearcnc_app_icon.ico"
        )
    endif()
    target_link_libraries(ClearCNC_Controller PRIVATE shell32)
endif()

