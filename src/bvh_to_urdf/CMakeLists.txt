cmake_minimum_required(VERSION 2.6)

include_directories("${SRC_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

find_package(Eigen3 REQUIRED)
add_executable(
        bvh_to_urdf
        main.cpp
)

