cmake_minimum_required(VERSION 3.16)
project(pyrvo
  VERSION 0.3.4
  LANGUAGES CXX)

find_package(pybind11 CONFIG REQUIRED)
add_subdirectory(src)
pybind11_add_module(pyrvo orca_bind.cpp)

target_link_libraries(pyrvo PRIVATE RVO)

target_compile_definitions(pyrvo PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS pyrvo DESTINATION pyrvo)

