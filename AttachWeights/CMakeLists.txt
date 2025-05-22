cmake_minimum_required(VERSION 3.10)
project(AttachWeights)

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "-O3 -Wall -fPIC")

# Detect OS
if(APPLE)
    add_definitions(-DOSX)
    set(TARGET_EXT Mac)
elseif(UNIX)
    add_definitions(-DLINUX)
    set(TARGET_EXT Linux)
endif()

include_directories(${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/Pinocchio)

link_directories(${CMAKE_BINARY_DIR}/Pinocchio)

set(BINARY_FILES
    stdafx.cpp
    AttachWeights.cpp
)

add_executable(AttachWeights ${BINARY_FILES})

target_link_libraries(AttachWeights
    pinocchio
    stdc++
)

target_link_libraries(AttachWeights
    ${CMAKE_BINARY_DIR}/Pinocchio/libpinocchio.a
    stdc++
)
