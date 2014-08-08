cmake_minimum_required(VERSION 2.6)

include_directories("${PROJECT_SOURCE_DIR}/include")
include_directories("${SRC_DIR}")
include_directories("${PQP_INCLUDE_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("${CDD_INCLUDE_DIR}")

LINK_DIRECTORIES(${CDD_LIBRARY_DIR})
LINK_DIRECTORIES(${PQP_LIBRARY_DIR})

add_executable(
        equilibrium_tests Main.cpp
)

TARGET_LINK_LIBRARIES(equilibrium_tests ${CDD_LIBRARY})
