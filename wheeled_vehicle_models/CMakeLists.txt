cmake_minimum_required(VERSION 3.10)
project(MultiProject LANGUAGES CXX)

# -----------------------------------------------------
# User options to select which projects to build
option(BUILD_11DOF "Build the 11dof project" OFF)
option(BUILD_11DOF_GPU "Build the 11dof-gpu project" OFF)
option(BUILD_18DOF "Build the 18dof project" OFF)
option(BUILD_18DOF_GPU "Build the 18dof-gpu project" OFF)
option(BUILD_24DOF "Build the 24dof project" OFF)
option(BUILD_24DOF_GPU "Build the 24dof-gpu project" OFF)

# -----------------------------------------------------
# Global options for subprojects
option(USE_OPENMP "Use OpenMP for parallelization" OFF)
option(USE_SUNDIALS "Use SUNDIALS library" OFF)
option(BUILD_PYTHON_MODULE "Build Python module" OFF)

# -----------------------------------------------------
# Set global C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------
# Enable CUDA language globally
if(BUILD_11DOF_GPU OR BUILD_18DOF_GPU OR BUILD_24DOF_GPU)
    enable_language(CUDA)
    find_package(CUDA 8.0 REQUIRED)
    message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}")
    message("We come into this")
endif()

# -----------------------------------------------------
# Optionally set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -ggdb3)
    add_definitions(-DDEBUG)
    set(CUDA_NVCC_FLAGS_DEBUG "-G -g" CACHE STRING "NVCC debug flags" FORCE)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
endif()

# -----------------------------------------------------
# Add subdirectories conditionally based on user options
if(BUILD_11DOF)
    add_subdirectory(11dof)
endif()

if(BUILD_11DOF_GPU)
    add_subdirectory(11dof-gpu)
endif()

if(BUILD_18DOF)
    add_subdirectory(18dof)
endif()

if(BUILD_18DOF_GPU)
    add_subdirectory(18dof-gpu)
endif()

if(BUILD_24DOF)
    add_subdirectory(24dof)
endif()

if(BUILD_24DOF_GPU)
    add_subdirectory(24dof-gpu)
endif()
