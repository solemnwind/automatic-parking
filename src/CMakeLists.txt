cmake_minimum_required(VERSION 3.23)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set (PYTHON_EXECUTABLE "/opt/miniconda3/envs/auto/bin/python")

project(reeds_shepp)

add_subdirectory(pybind11)
pybind11_add_module(reeds_shepp reeds_shepp.h reeds_shepp.cpp)

add_custom_command(
    TARGET ${PROJECT_NAME}  POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_BINARY_DIR} ../build
)