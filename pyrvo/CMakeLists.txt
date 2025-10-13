find_package(pybind11 CONFIG REQUIRED)
add_subdirectory(src)
pybind11_add_module(pyrvo orca_bind.cpp)

target_link_libraries(pyrvo PRIVATE RVO)

target_compile_definitions(pyrvo PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS pyrvo DESTINATION pyrvo)

