cmake_minimum_required(VERSION 3.15...4.0)
project(pyrvo LANGUAGES CXX VERSION ${PROJECT_VERSION})

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

add_subdirectory(src)

pybind11_add_module(pyrvo orca_bind.cpp)

target_link_libraries(pyrvo PRIVATE RVO)

target_compile_definitions(pyrvo PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS pyrvo DESTINATION .)

