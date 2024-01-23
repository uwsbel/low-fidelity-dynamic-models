cmake_minimum_required(VERSION 3.18)
project(testing)
# -----------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BINARY_DIR  ${CMAKE_SOURCE_DIR}/build)

# -----------------------------------------------------

# Set Cmake build type to by default to release
if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
endif()

# If specified set the CMake mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -ggdb3)
    add_definitions(-DDEBUG)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
endif()

# -----------------------------------------------------
# OpenMp
find_package(OpenMP REQUIRED)
add_definitions(-DUSE_OPENMP)

# -----------------------------------------------------
# Sundials
find_package(SUNDIALS REQUIRED)


# -----------------------------------------------------
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

set(CPU_MODEL_FILES
  ${CMAKE_SOURCE_DIR}/../18dof/dof18.h
  ${CMAKE_SOURCE_DIR}/../18dof/dof18.cpp
  ${CMAKE_SOURCE_DIR}/../11dof/dof11.h
  ${CMAKE_SOURCE_DIR}/../11dof/dof11.cpp
  ${CMAKE_SOURCE_DIR}/../24dof/dof24.h
  ${CMAKE_SOURCE_DIR}/../24dof/dof24.cpp

)

set(CPU_UTILS_FILES
    ${CMAKE_SOURCE_DIR}/../utils/utils.h
    ${CMAKE_SOURCE_DIR}/../utils/utils.cpp
)

# Use half implicit solver by default
set(CPU_SOLVER_FILES
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_halfImplicit.h
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_halfImplicit.cpp
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_sundials.h
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_sundials.cpp
  ${CMAKE_SOURCE_DIR}/../11dof/dof11_halfImplicit.h
  ${CMAKE_SOURCE_DIR}/../11dof/dof11_halfImplicit.cpp
  ${CMAKE_SOURCE_DIR}/../11dof/dof11_sundials.h
  ${CMAKE_SOURCE_DIR}/../11dof/dof11_sundials.cpp
  ${CMAKE_SOURCE_DIR}/../24dof/dof24_halfImplicit.h
  ${CMAKE_SOURCE_DIR}/../24dof/dof24_halfImplicit.cpp
  ${CMAKE_SOURCE_DIR}/../24dof/dof24_sundials.h
  ${CMAKE_SOURCE_DIR}/../24dof/dof24_sundials.cpp
)

# Test programs to build
set(CPU_TEST_PROGRAMS
  18dof
  11dof
  24dof
)
enable_testing() 


# -----------------------------------------------------

include_directories(${CMAKE_SOURCE_DIR}/../18dof)
include_directories(${CMAKE_SOURCE_DIR}/../11dof)
include_directories(${CMAKE_SOURCE_DIR}/../24dof)
include_directories(${CMAKE_SOURCE_DIR}/../utils)
include_directories(${CMAKE_SOURCE_DIR}/../third_party)

# -----------------------------------------------------
# Build CPU Tests
# -----------------------------------------------------
# CPU tests
foreach(PROGRAM ${CPU_TEST_PROGRAMS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} ${CMAKE_SOURCE_DIR}/${PROGRAM}.cpp ${CPU_MODEL_FILES} ${CPU_UTILS_FILES} ${CPU_SOLVER_FILES})

  # Link OpenMP
  target_link_libraries(${PROGRAM} PUBLIC OpenMP::OpenMP_CXX)

  # Link sundials
  target_link_libraries(${PROGRAM} PUBLIC SUNDIALS::cvodes SUNDIALS::nvecserial SUNDIALS::sunlinsoldense SUNDIALS::sunmatrixdense )

  # Link to GoogleTests
  target_link_libraries(${PROGRAM} PUBLIC GTest::gtest_main)
endforeach()

include(GoogleTest)
gtest_discover_tests(18dof)
gtest_discover_tests(11dof)
gtest_discover_tests(24dof)

# -----------------------------------------------------
# GPU tests dependencies
option(BUILD_GPU_TESTS "Enable building of GPU tests" OFF)
if(BUILD_GPU_TESTS)
  enable_language(CUDA)
  find_package(CUDA 8.0 REQUIRED)
  message(STATUS "Found CUDA ${CUDA_VERSION_STRING} at ${CUDA_TOOLKIT_ROOT_DIR}")
  set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}")

  set(GPU_MODEL_FILES
  ${CMAKE_SOURCE_DIR}/../18dof-gpu/dof18_gpu.cuh
  ${CMAKE_SOURCE_DIR}/../18dof-gpu/dof18_gpu.cu
  ${CMAKE_SOURCE_DIR}/../11dof-gpu/dof11_gpu.cuh
  ${CMAKE_SOURCE_DIR}/../11dof-gpu/dof11_gpu.cu
  ${CMAKE_SOURCE_DIR}/../24dof-gpu/dof24_gpu.cuh
  ${CMAKE_SOURCE_DIR}/../24dof-gpu/dof24_gpu.cu

  )

  set(GPU_UTILS_FILES
      ${CMAKE_SOURCE_DIR}/../utils/utils_gpu.cuh
      ${CMAKE_SOURCE_DIR}/../utils/utils_gpu.cu
  )

  # Use half implicit solver by default
  set(GPU_SOLVER_FILES
    ${CMAKE_SOURCE_DIR}/../18dof-gpu/dof18_halfImplicit_gpu.cuh
    ${CMAKE_SOURCE_DIR}/../18dof-gpu/dof18_halfImplicit_gpu.cu
    ${CMAKE_SOURCE_DIR}/../11dof-gpu/dof11_halfImplicit_gpu.cuh
    ${CMAKE_SOURCE_DIR}/../11dof-gpu/dof11_halfImplicit_gpu.cu
    ${CMAKE_SOURCE_DIR}/../24dof-gpu/dof24_halfImplicit_gpu.cuh
    ${CMAKE_SOURCE_DIR}/../24dof-gpu/dof24_halfImplicit_gpu.cu
  )

  # Test programs to build
  set(GPU_TEST_PROGRAMS
    18dof_gpu
    11dof_gpu
    24dof_gpu
  )

  include_directories(${CMAKE_SOURCE_DIR}/../18dof-gpu)
  include_directories(${CMAKE_SOURCE_DIR}/../11dof-gpu)
  include_directories(${CMAKE_SOURCE_DIR}/../24dof-gpu)

  # -----------------------------------------------------
  # Build GPU Tests
  # -----------------------------------------------------
  foreach(PROGRAM ${GPU_TEST_PROGRAMS})
    message(STATUS "...add ${PROGRAM}")
    add_executable(${PROGRAM} ${CMAKE_SOURCE_DIR}/${PROGRAM}.cu ${GPU_UTILS_FILES} ${GPU_MODEL_FILES} ${GPU_SOLVER_FILES})
    # Enable separate compilation for CUDA
    set_property(TARGET ${PROGRAM} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
    # Set architecure based on this link - https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
    set_target_properties(${PROGRAM} PROPERTIES CUDA_ARCHITECTURES "60;61;70;75;80;86;89;90")
    # Link to GoogleTests
    target_link_libraries(${PROGRAM} PUBLIC GTest::gtest_main)
  endforeach()

  gtest_discover_tests(18dof_gpu)
  gtest_discover_tests(11dof_gpu)
  gtest_discover_tests(24dof_gpu)
endif()

