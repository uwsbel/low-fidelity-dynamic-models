cmake_minimum_required(VERSION 3.12)
project(d18dof)
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

# Option to use openMP for parallelization
option(USE_OPENMP "Use OpenMP for Jacobian" OFF)
if(USE_OPENMP)
    find_package(OpenMP REQUIRED)
    add_definitions(-DUSE_OPENMP)
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

# -----------------------------------------------------

# Option to build Python module
option(BUILD_PYTHON_MODULE "Build Python module" OFF)
if(BUILD_PYTHON_MODULE)
    message(STATUS "Building Python module")
    add_subdirectory(py)
endif()


set(MODEL_FILES
    dof18.h
    dof18.cpp
)

set(UTILS_FILES
    ${CMAKE_SOURCE_DIR}/../utils/utils.h
    ${CMAKE_SOURCE_DIR}/../utils/utils.cpp
)

# Use half implicit solver by default
set(SOLVER_FILES
  dof18_halfImplicit.h
  dof18_halfImplicit.cpp
)
# Add sundials solver if required
if(USE_SUNDIALS)
  set(SOLVER_FILES ${SOLVER_FILES}
      dof18_sundials.h
      dof18_sundials.cpp
  )
endif()

# Build half implicit demos by default
# HMMWV demos
set(HMMWV_PROGRAMS
    demo_hmmwv_hi
    demo_hmmwv_hi_step
    demo_hmmwv_hi_stepWithJac
)
# Sedan demos
set(SEDAN_PROGRAMS
    demo_sedan_hi
)

# Add sundials demos if required
if(USE_SUNDIALS)
  set(HMMWV_PROGRAMS ${HMMWV_PROGRAMS}
      demo_hmmwv_sundials_withFSA
      demo_hmmwv_sundials
  )
  set(SEDAN_PROGRAMS ${SEDAN_PROGRAMS}
      demo_sedan_sundials
  )
endif()

# Add snopt demos if required
if(USE_SNOPT)
  set(HMMWV_PROGRAMS ${HMMWV_PROGRAMS}
      demo_snopt
  )
endif()

# -----------------------------------------------------

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../utils)
include_directories(${CMAKE_SOURCE_DIR}/../third_party)

# -----------------------------------------------------
## Build HMMWV demos
# -----------------------------------------------------
foreach(PROGRAM ${HMMWV_PROGRAMS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} ${CMAKE_SOURCE_DIR}/demos/HMMWV/${PROGRAM}.cpp ${MODEL_FILES} ${UTILS_FILES} ${SOLVER_FILES})

  if(USE_OPENMP)
    target_link_libraries(${PROGRAM} PUBLIC OpenMP::OpenMP_CXX)
  endif()

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

# -----------------------------------------------------
## Build Sedan demos
# -----------------------------------------------------
foreach(PROGRAM ${SEDAN_PROGRAMS})
  message(STATUS "...add ${PROGRAM}")
  add_executable(${PROGRAM} ${CMAKE_SOURCE_DIR}/demos/Sedan/${PROGRAM}.cpp ${MODEL_FILES} ${UTILS_FILES} ${SOLVER_FILES})

  if(USE_OPENMP)
    target_link_libraries(${PROGRAM} PUBLIC OpenMP::OpenMP_CXX)
  endif()
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

