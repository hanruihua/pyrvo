cmake_minimum_required(VERSION 3.16)
project(pyrvo
  VERSION 0.3.4
  LANGUAGES CXX)

find_package(pybind11 CONFIG REQUIRED)

add_subdirectory(src)
pybind11_add_module(pyrvo orca_bind.cpp)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

target_link_libraries(pyrvo PUBLIC RVO)

# Ensure the Python extension looks for libRVO next to itself
if(UNIX AND BUILD_SHARED_LIBS)
  set_target_properties(pyrvo PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/.libs:$ORIGIN"
    BUILD_RPATH "$ORIGIN/.libs:$ORIGIN")
endif()

install(TARGETS pyrvo DESTINATION .)

# Bundle the native library inside the wheel so audit tools can find it
if(UNIX AND BUILD_SHARED_LIBS)
  # Copy the built shared library next to the extension under .libs
  install(FILES $<TARGET_FILE:RVO> DESTINATION pyrvo/.libs)
endif()

