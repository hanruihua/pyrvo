#include <pybind11/pybind11.h>

namespace py = pybind11;

int add(int i, int j) {
    return i + j;
}

int multiply(int i, int j) {
    return i * j;
}

PYBIND11_MODULE(add_example, m, py::mod_gil_not_used()) {
    m.doc() = "pybind11 example plugin"; // optional module docstring
    m.def("add", &add, "A function that adds two numbers");
    m.def("multiply", &multiply, "A function that multiplies two numbers");
}