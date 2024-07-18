cmake_minimum_required(VERSION 3.23)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -flto=auto")

set (PYTHON_EXECUTABLE "/opt/miniconda3/envs/automatic-parking/bin/python")

project(_reeds_shepp)

add_subdirectory(pybind11)
pybind11_add_module(_reeds_shepp reeds_shepp.h reeds_shepp.cpp)

set(DEST_DIR ${CMAKE_SOURCE_DIR}/../../scripts/models)

add_custom_command(TARGET _reeds_shepp POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:_reeds_shepp> ${DEST_DIR}
)
