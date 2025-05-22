cmake_minimum_required(VERSION 3.10)
project(DemoUI)

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "-Wall -O3")

include_directories(../Pinocchio)

link_directories(../Pinocchio)

set(SOURCE_FILES
    demoUI.cpp
    MyWindow.cpp
    defmesh.cpp
    processor.cpp
    motion.cpp
    filter.cpp
)

add_executable(DemoUI ${SOURCE_FILES})

target_link_libraries(DemoUI
    pinocchio
    OpenGL
    fltk
    fltk_gl
    m
)
