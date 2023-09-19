cmake_minimum_required(VERSION 3.2)
project(d18dof)

# -----------------------------------------------------

set(CMAKE_BINARY_DIR  ${CMAKE_SOURCE_DIR}/build)

# -----------------------------------------------------

# Set Cmake build type to by default to release
if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
endif()

# If specified set the CMake mode
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -ggdb3)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
endif()

# -----------------------------------------------------

# Option to use sundials
option(USE_SUNDIALS "Use SUNDIALS library" OFF)
if(USE_SUNDIALS)
  find_package(SUNDIALS REQUIRED)
endif()

# -----------------------------------------------------

# Option to use snopt
option(USE_SNOPT "Use SNOPT library" OFF)
if(USE_SNOPT)
    set(SNOPT_LIBRARY "" CACHE FILEPATH "Path to SNOPT library")
    set(SNOPT_INCLUDE_DIR "" CACHE PATH "Path to SNOPT headers")
endif()


set(MODEL_FILES
    dof18.h
    dof18.cpp
)

set(UTILS_FILES
    ${CMAKE_SOURCE_DIR}/../utils/utils.h
    ${CMAKE_SOURCE_DIR}/../utils/utils.cpp
)

# Use sundials or half implicit solver
if(USE_SUNDIALS)
  set(SOLVER_FILES
      dof18_sundials.h
      dof18_sundials.cpp
  )
else()
  set(SOLVER_FILES
      dof18_halfImplicit.h
      dof18_halfImplicit.cpp
  )
endif()

# Append to this list the name of the programs you want to build
set(PROGRAMS
    demo_sedan_sundials
    demo_hmmwv_sundials
)

if(USE_SNOPT)
  set(PROGRAMS ${PROGRAMS}
      demo_snopt.cpp
  )
endif()

# -----------------------------------------------------

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../utils)
include_directories(${CMAKE_SOURCE_DIR}/../third_party)

foreach(PROGRAM ${PROGRAMS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} ${CMAKE_SOURCE_DIR}/demos/${PROGRAM}.cpp ${MODEL_FILES} ${UTILS_FILES} ${SOLVER_FILES})

  # Link sundials libraries if required
  if(USE_SUNDIALS)
    target_link_libraries(${PROGRAM} PUBLIC SUNDIALS::cvodes SUNDIALS::nvecserial SUNDIALS::sunlinsoldense SUNDIALS::sunmatrixdense )
  endif()

  # Link snopt libraries if required
  if(USE_SNOPT)
    target_include_directories(${PROGRAM} PUBLIC ${SNOPT_INCLUDE_DIR})
    target_link_libraries(${PROGRAM} PUBLIC ${SNOPT_LIBRARY})
  endif()

endforeach()

