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

set(CPU_MODEL_FILES
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/18dof/dof18.h
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/18dof/dof18.cpp
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/11dof/dof11.h
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/11dof/dof11.cpp
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/24dof/dof24.h
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/24dof/dof24.cpp

)

set(CPU_UTILS_FILES
    ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/utils/utils.h
    ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/utils/utils.cpp
)

# Use half implicit solver by default
set(CPU_SOLVER_FILES
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/18dof/dof18_halfImplicit.h
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/18dof/dof18_halfImplicit.cpp
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/11dof/dof11_halfImplicit.h
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/11dof/dof11_halfImplicit.cpp
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/24dof/dof24_halfImplicit.h
  ${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/24dof/dof24_halfImplicit.cpp
)

# Test programs to build
set(CPU_TEST_PROGRAMS
  accTest
  dlTest
)

# -----------------------------------------------------

include_directories(${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/18dof)
include_directories(${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/11dof)
include_directories(${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/24dof)
include_directories(${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/utils)
include_directories(${CMAKE_SOURCE_DIR}/../../wheeled_vehicle_models/third_party)

# -----------------------------------------------------
# Build CPU Tests
# -----------------------------------------------------
# CPU tests
foreach(PROGRAM ${CPU_TEST_PROGRAMS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} ${CMAKE_SOURCE_DIR}/${PROGRAM}.cpp ${CPU_MODEL_FILES} ${CPU_UTILS_FILES} ${CPU_SOLVER_FILES})
endforeach()