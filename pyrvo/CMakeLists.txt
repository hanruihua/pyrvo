cmake_minimum_required(VERSION 3.16)
project(pyrvo
  VERSION 0.3.4
  LANGUAGES CXX)

find_package(pybind11 CONFIG REQUIRED)
find_package(RVO MODULE REQUIRED)

add_subdirectory(src)
pybind11_add_module(pyrvo orca_bind.cpp)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

target_link_libraries(pyrvo PRIVATE RVO)

install(TARGETS pyrvo DESTINATION .)

