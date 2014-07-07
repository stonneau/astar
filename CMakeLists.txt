cmake_minimum_required(VERSION 2.6)
project(astar)


set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/build/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")


add_subdirectory (src/prm)
add_subdirectory (src/tests/astar)

