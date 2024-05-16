cmake_minimum_required(VERSION 3.10)
project(d18dof_gpu LANGUAGES CXX CUDA)
# -----------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BINARY_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/build)

# -----------------------------------------------------
enable_language(CUDA)
find_package(CUDA 8.0 REQUIRED)
message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}")


# -----------------------------------------------------

# Set Cmake build type to by default to release
if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
endif()

# If specified set the CMake mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -ggdb3)
    add_definitions(-DDEBUG)

    set(CUDA_NVCC_FLAGS_DEBUG "-G -g" CACHE STRING "NVCC debug flags" FORCE)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
endif()



set(MODEL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/dof18_gpu.cuh
    ${CMAKE_CURRENT_SOURCE_DIR}/dof18_gpu.cu
)

set(UTILS_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/utils_gpu.cuh
    ${CMAKE_CURRENT_SOURCE_DIR}/../utils/utils_gpu.cu
)

# Only half implicit solver available for GPU
set(SOLVER_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dof18_halfImplicit_gpu.cuh
  ${CMAKE_CURRENT_SOURCE_DIR}/dof18_halfImplicit_gpu.cu
)

# Build the demos
set(HMMWV_PROGRAMS
    demo_hmmwv_18gpu
    demo_hmmwv_varControls_18gpu
    demo_hmmwv_step_18gpu
    demo_hmmwv_controlsFunctor_18gpu
)

# -----------------------------------------------------

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/utils)
include_directories(${CMAKE_SOURCE_DIR}/third_party)


# -----------------------------------------------------
## Build HMMWV demos
# -----------------------------------------------------
foreach(PROGRAM ${HMMWV_PROGRAMS})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} ${CMAKE_CURRENT_SOURCE_DIR}/demos/HMMWV/${PROGRAM}.cu ${UTILS_FILES} ${MODEL_FILES} ${SOLVER_FILES})
    # Enable separate compilation for CUDA
    set_property(TARGET ${PROGRAM} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
    # Set architecure based on this link - https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
    set_target_properties(${PROGRAM} PROPERTIES CUDA_ARCHITECTURES "60;61;70;75;80;86;89;90")
endforeach()