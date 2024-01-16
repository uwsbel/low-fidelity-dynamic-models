cmake_minimum_required(VERSION 3.12)
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

set(MODEL_FILES
  ${CMAKE_SOURCE_DIR}/../18dof/dof18.h
  ${CMAKE_SOURCE_DIR}/../18dof/dof18.cpp
)

set(UTILS_FILES
    ${CMAKE_SOURCE_DIR}/../utils/utils.h
    ${CMAKE_SOURCE_DIR}/../utils/utils.cpp
)

# Use half implicit solver by default
set(SOLVER_FILES
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_halfImplicit.h
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_halfImplicit.cpp
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_sundials.h
  ${CMAKE_SOURCE_DIR}/../18dof/dof18_sundials.cpp
)

# Test programs to build
set(TEST_PROGRAMS
  18dof
)
enable_testing() 


# -----------------------------------------------------

include_directories(${CMAKE_SOURCE_DIR}/../18dof)
include_directories(${CMAKE_SOURCE_DIR}/../utils)
include_directories(${CMAKE_SOURCE_DIR}/../third_party)

# -----------------------------------------------------
# Build Tests
# -----------------------------------------------------
foreach(PROGRAM ${TEST_PROGRAMS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} ${CMAKE_SOURCE_DIR}/${PROGRAM}.cpp ${MODEL_FILES} ${UTILS_FILES} ${SOLVER_FILES})

  # Link OpenMP
  target_link_libraries(${PROGRAM} PUBLIC OpenMP::OpenMP_CXX)

  # Link sundials
  target_link_libraries(${PROGRAM} PUBLIC SUNDIALS::cvodes SUNDIALS::nvecserial SUNDIALS::sunlinsoldense SUNDIALS::sunmatrixdense )

  # Link to GoogleTests
  target_link_libraries(${PROGRAM} PUBLIC GTest::gtest_main)


endforeach()

include(GoogleTest)
gtest_discover_tests(18dof)
