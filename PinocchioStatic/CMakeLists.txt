project(PinocchioStatic)

# Gather all source files
set(PINOCCHIO_SRC
    ../Pinocchio/attachment.cpp
    ../Pinocchio/discretization.cpp
    ../Pinocchio/embedding.cpp
    ../Pinocchio/graphutils.cpp
    ../Pinocchio/indexer.cpp
    ../Pinocchio/intersector.cpp
    ../Pinocchio/lsqSolver.cpp
    ../Pinocchio/matrix.cpp
    ../Pinocchio/mesh.cpp
    ../Pinocchio/pinocchioApi.cpp
    ../Pinocchio/refinement.cpp
    ../Pinocchio/skeleton.cpp
)

# Gather all header files (optional but useful for IDEs)
set(PINOCCHIO_HEADERS
    ../Pinocchio/attachment.h
    ../Pinocchio/debugging.h
    ../Pinocchio/deriv.h
    ../Pinocchio/dtree.h
    ../Pinocchio/graphutils.h
    ../Pinocchio/hashutils.h
    ../Pinocchio/indexer.h
    ../Pinocchio/intersector.h
    ../Pinocchio/lsqSolver.h
    ../Pinocchio/mathutils.h
    ../Pinocchio/matrix.h
    ../Pinocchio/mesh.h
    ../Pinocchio/multilinear.h
    ../Pinocchio/Pinocchio.h
    ../Pinocchio/pinocchioApi.h
    ../Pinocchio/pointprojector.h
    ../Pinocchio/quaddisttree.h
    ../Pinocchio/rect.h
    ../Pinocchio/skeleton.h
    ../Pinocchio/transform.h
    ../Pinocchio/utils.h
    ../Pinocchio/vector.h
    ../Pinocchio/vecutils.h
)

# Define the static library
add_library(PinocchioStatic STATIC ${PINOCCHIO_SRC} ${PINOCCHIO_HEADERS})

# Include directories
target_include_directories(PinocchioStatic PUBLIC
    ${CMAKE_SOURCE_DIR}/Pinocchio
)

# Define preprocessor macros
target_compile_definitions(PinocchioStatic PRIVATE
    _LIB
    PINOCCHIO_STATIC
)

# Compiler options for debug and release
target_compile_options(PinocchioStatic PRIVATE
    $<$<CONFIG:Debug>:/Od /Zi /RTC1>           # Optimization off, debug info, runtime checks
    $<$<CONFIG:Release>:/O2 /DNDEBUG>          # Optimization on, no debug
)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_definitions(PinocchioStatic PRIVATE _DEBUG)
    target_compile_options(PinocchioStatic PRIVATE -g -O0 -Wall)
else()
    target_compile_definitions(PinocchioStatic PRIVATE NDEBUG)
    target_compile_options(PinocchioStatic PRIVATE -O2 -Wall)
endif()

# Set the runtime library (matches Visual Studio project: /MDd for Debug, /MD for Release)
# Use MSVC_RUNTIME_LIBRARY only with CMake 3.15+
if(MSVC)
    set_property(TARGET PinocchioStatic PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

