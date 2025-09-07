cmake_minimum_required(VERSION 3.15...4.0)
project(pyorca LANGUAGES CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

add_subdirectory(src)

pybind11_add_module(pyorca orca_bind.cpp)
install(TARGETS pyorca DESTINATION .)

target_link_libraries(pyorca PRIVATE RVO)

# set_target_properties(pyorca PROPERTIES PREFIX "" OUTPUT_NAME "pyorca")
# if(WIN32)
#   set_target_properties(pyorca PROPERTIES SUFFIX ".pyd")
# endif()
# install(TARGETS pyorca DESTINATION .)

