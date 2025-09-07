cmake_minimum_required(VERSION 3.15...4.0)
project(pyrvo LANGUAGES CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

add_subdirectory(src)

pybind11_add_module(pyrvo orca_bind.cpp)
install(TARGETS pyrvo DESTINATION .)

target_link_libraries(pyrvo PRIVATE RVO)

# set_target_properties(pyrvo PROPERTIES PREFIX "" OUTPUT_NAME "pyrvo")
# if(WIN32)
#   set_target_properties(pyrvo PROPERTIES SUFFIX ".pyd")
# endif()
# install(TARGETS pyrvo DESTINATION .)

