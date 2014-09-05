cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${SRC_DIR})
include_directories("${SRC_TOOLS_DIR})
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

PROJECT(prmpath)

LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    Robot.h
    Robot.cpp
    JointConstraint.h
    JointConstraint.cpp
    Jacobian.h
    Jacobian.cpp
    ROM.h
    ROM.cpp
    sampling/Sample.h
    sampling/Sample.cpp
        ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
        ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
)

add_library(prmpath ${SOURCES})
TARGET_LINK_LIBRARIES(prmpath prmcollision)
TARGET_LINK_LIBRARIES(prmpath prm)
TARGET_LINK_LIBRARIES(prmpath PQP)

SET_TARGET_PROPERTIES(prmpath PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(prmpath PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
