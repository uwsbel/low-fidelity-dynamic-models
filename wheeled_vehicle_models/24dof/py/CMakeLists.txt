message(STATUS "\n==== dof24 Python module ====\n")

set(LIBRARY_NAME "pydof24")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ---------------------------------------------------------------------
# Find Python
# ---------------------------------------------------------------------

message(STATUS "...find Python")
# find_package(PythonInterp REQUIRED)
find_package(Python3 COMPONENTS Interpreter)
find_package(PythonLibs REQUIRED)
message(STATUS "Found python: ${Python3_EXECUTABLE}")
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIR})

# Option to use openMP for parallelization
if(${USE_OPENMP})
    find_package(OpenMP REQUIRED)
endif()

# ---------------------------------------------------------------------
# Find SWIG
# ---------------------------------------------------------------------

message(STATUS "...find SWIG")
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}")
set(CMAKE_SWIG_FLAGS "")

# ---------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../)
include_directories(${CMAKE_SOURCE_DIR}/utils)
include_directories(${CMAKE_SOURCE_DIR}/third_party)

# ---------------------------------------------------------------------

# For some reason still need to put these here
set(MODEL_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../dof24.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../dof24.cpp
)

set(UTILS_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../utils/utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../utils/utils.cpp
)

# Use half implicit solver by default
set(SOLVER_FILES
   ${CMAKE_CURRENT_SOURCE_DIR}/../dof24_halfImplicit.h
   ${CMAKE_CURRENT_SOURCE_DIR}/../dof24_halfImplicit.cpp
)
# Add sundials solver if required
if(${USE_SUNDIALS})
  set(SOLVER_FILES ${SOLVER_FILES}
  ${CMAKE_CURRENT_SOURCE_DIR}/../dof24_sundials.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../dof24_sundials.cpp
  )
endif()



# ---------------------------------------------------------------------

if(${USE_SUNDIALS})
    SET_SOURCE_FILES_PROPERTIES(dof24Sundials.i PROPERTIES CPLUSPLUS ON)
    SWIG_ADD_LIBRARY(${LIBRARY_NAME} LANGUAGE python SOURCES ${MODEL_FILES} ${UTILS_FILES} ${SOLVER_FILES} dof24Sundials.i)

    # Link to OpenMP if required
    if(USE_OPENMP)
        SWIG_LINK_LIBRARIES(${LIBRARY_NAME} ${PYTHON_LIBRARIES} SUNDIALS::cvodes SUNDIALS::nvecserial SUNDIALS::sunlinsoldense SUNDIALS::sunmatrixdense OpenMP::OpenMP_CXX)
    else()
        SWIG_LINK_LIBRARIES(${LIBRARY_NAME} ${PYTHON_LIBRARIES} SUNDIALS::cvodes SUNDIALS::nvecserial SUNDIALS::sunlinsoldense SUNDIALS::sunmatrixdense)
    endif()


else()
    SET_SOURCE_FILES_PROPERTIES(dof24.i PROPERTIES CPLUSPLUS ON)
    SWIG_ADD_LIBRARY(${LIBRARY_NAME} LANGUAGE python SOURCES ${MODEL_FILES} ${UTILS_FILES} ${SOLVER_FILES} dof24.i)

    # Link to OpenMP if required
    if(${USE_OPENMP})
        SWIG_LINK_LIBRARIES(${LIBRARY_NAME} ${PYTHON_LIBRARIES} OpenMP::OpenMP_CXX)
    else()
        SWIG_LINK_LIBRARIES(${LIBRARY_NAME} ${PYTHON_LIBRARIES})
    endif()


endif()

# ---------------------------------------------------------------------

# This is done to keep a nice name and also to ensure the library is build
# in the same place as the c++ demos
set_target_properties(${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME} PROPERTIES 
                        OUTPUT_NAME "${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME}"
                        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")