cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${SRC_DIR})
include_directories("${SRC_TOOLS_DIR})
include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("${CDD_INCLUDE_DIR}")

PROJECT(equilibrium)


if ( MSVC )
	SET(CMAKE_DEBUG_POSTFIX d)
endif ( MSVC )
	
set(SOURCES
    Test.h
    Test.cpp
    ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
    ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
)


LINK_DIRECTORIES(${PQP_LIBRARY_DIR})
LINK_DIRECTORIES(${CDD_LIBRARY_DIR})

add_library(equilibrium ${SOURCES})

TARGET_LINK_LIBRARIES(equilibrium PQP)
TARGET_LINK_LIBRARIES(equilibrium CDD)

SET_TARGET_PROPERTIES(equilibrium PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
SET_TARGET_PROPERTIES(equilibrium PROPERTIES ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
