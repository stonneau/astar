cmake_minimum_required(VERSION 2.6)

include_directories("${SRC_DIR}")
include_directories("${SRC_TOOLS_DIR}")
include_directories("${EIGEN3_INCLUDE_DIR}")

find_package(Eigen3 REQUIRED)
add_executable(
        bvh_to_urdf
        main.cpp
        create_obj_box.h
        create_obj_box.cpp
        ${SRC_TOOLS_DIR}/MatrixDefs.h ${SRC_TOOLS_DIR}/MatrixDefs.cpp
        ${SRC_TOOLS_DIR}/MatrixDefsInternal.h
)

