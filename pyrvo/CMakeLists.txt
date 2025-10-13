find_package(pybind11 CONFIG REQUIRED)
add_subdirectory(src)
pybind11_add_module(pyrvo orca_bind.cpp)

target_link_libraries(pyrvo PRIVATE RVO)

# Set RPATH for shared libraries on Unix systems
if(UNIX AND BUILD_SHARED_LIBS)
  set_target_properties(pyrvo PROPERTIES
    INSTALL_RPATH_USE_LINK_PATH TRUE
    INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib")
endif()

target_compile_definitions(pyrvo PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS pyrvo DESTINATION pyrvo)

