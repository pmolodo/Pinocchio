cmake_minimum_required(VERSION 3.10)
project(PinocchioProject)

add_compile_options(-Wno-deprecated)

# Define directories
set(DIRS
    Pinocchio
    AttachWeights
    DemoUI
    PinocchioStatic
)

# Add subdirectories (analogous to `makePerDir`)
foreach(DIR ${DIRS})
    add_subdirectory(${DIR})
endforeach()

# Optional: Add a custom clean target (though `make clean` is usually provided automatically)

