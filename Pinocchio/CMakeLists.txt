cmake_minimum_required(VERSION 3.10)
project(pinocchio)

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "-O3 -Wall -fPIC")

set(SOURCE_FILES
    attachment.cpp
    discretization.cpp
    indexer.cpp
    lsqSolver.cpp
    mesh.cpp
    graphutils.cpp
    intersector.cpp
    matrix.cpp
    skeleton.cpp
    embedding.cpp
    pinocchioApi.cpp
    refinement.cpp
)

set(OBJECT_FILES
    attachment.o
    discretization.o
    indexer.o
    lsqSolver.o
    mesh.o
    graphutils.o
    intersector.o
    matrix.o
    skeleton.o
    embedding.o
    pinocchioApi.o
    refinement.o
)

add_library(pinocchio STATIC ${SOURCE_FILES})
target_link_libraries(pinocchio m)
