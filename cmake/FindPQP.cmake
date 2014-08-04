# Base Io build system
# Written by Steve Tonneau steveteonneau@hotmail.fr
#
# Find PQP
FIND_PATH(PQP_INCLUDE_DIR PQP/PQP.h
    /usr/include
    /usr/local/include
)

SET(PQP_NAMES ${PQP_NAMES} PQP libPQP)
FIND_LIBRARY(PQP_LIBRARY NAMES ${PQP_NAMES} PATH)

IF(PQP_LIBRARY)
    MESSAGE(STATUS "Found PQP library: ${PQP_LIBRARY}")
ELSE(PQP_LIBRARY)
    MESSAGE(STATUS "Coulddn't find PQP library: ${PQP_LIBRARY}")
ENDIF(PQP_LIBRARY)

IF(PQP_INCLUDE_DIR AND PQP_LIBRARY)
SET(PQP_FOUND TRUE CACHE STRING "Whether PQP was found or not")
ENDIF(PQP_INCLUDE_DIR AND PQP_LIBRARY)

IF(PQP_FOUND)
    SET(CMAKE_C_FLAGS "-DdSINGLE")
IF(NOT PQP_FIND_QUIETLY)
MESSAGE(STATUS "Found PQP: ${PQP_LIBRARY}")
ENDIF (NOT PQP_FIND_QUIETLY)
ELSE(PQP_FOUND)
IF(PQP_FIND_REQUIRED)
MESSAGE(FATAL_ERROR "Could not find PQP")
ENDIF(PQP_FIND_REQUIRED)
ENDIF(PQP_FOUND)
