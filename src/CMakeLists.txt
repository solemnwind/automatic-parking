cmake_minimum_required(VERSION 3.23)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -flto=auto")

project(automatic_parking)

set (PYTHON_EXECUTABLE "/opt/miniconda3/envs/automatic-parking/bin/python")

add_subdirectory(pybind11)
add_subdirectory(occupancy_map_cpp)
add_subdirectory(reeds_shepp_cpp)
