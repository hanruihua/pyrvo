#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <vector>
#include "src/RVOSimulator.h"
#include "src/Vector2.h"
#include "src/Line.h"

namespace py = pybind11;
using RVOSIM = RVO::RVOSimulator;


py::tuple vector_to_tuple(const RVO::Vector2 &v) {
    return py::make_tuple(v.x(), v.y());
}

RVO::Vector2 tuple_to_vector(const py::tuple &t) {

    if (t.size() != 2) {
        throw std::runtime_error("Tuple must have exactly 2 elements");
    }

    return RVO::Vector2(t[0].cast<float>(), t[1].cast<float>());
}

RVO::Vector2 sequence_to_vector(const py::object &obj) {
    py::sequence seq = py::reinterpret_borrow<py::sequence>(obj);
    if (py::len(seq) != 2) {
        throw std::runtime_error("Sequence must have exactly 2 elements");
    }
    return RVO::Vector2(py::float_(seq[0]).cast<float>(), py::float_(seq[1]).cast<float>());
}

std::vector<RVO::Vector2> iterable_to_vector2_list(const py::object &iterable) {
    std::vector<RVO::Vector2> out;
    for (py::handle item : py::reinterpret_borrow<py::iterable>(iterable)) {
        out.emplace_back(sequence_to_vector(py::reinterpret_borrow<py::object>(item)));
    }
    return out;
}


PYBIND11_MODULE(pyrvo, m) {
    m.doc() = "ORCA algorithm python bindings"; // optional module docstring

    py::class_<RVO::Vector2>(m, "Vector2")
        .def(py::init<>())
        .def(py::init<float, float>())
        .def_property_readonly("x", &RVO::Vector2::x)
        .def_property_readonly("y", &RVO::Vector2::y)
        .def("to_tuple", &vector_to_tuple)
        .def_static("from_tuple", &tuple_to_vector)
        .def_static("from_list", &sequence_to_vector)
        .def("__repr__", [](const RVO::Vector2& v) {
            return "Vector2(" + std::to_string(v.x()) + ", " + std::to_string(v.y()) + ")";
        });

    py::class_<RVOSIM>(m, "RVOSimulator")
        .def(py::init<>())
        .def(py::init<float, float, std::size_t, float, float, float, float>())
        .def(py::init<float, float, std::size_t, float, float, float, float, const RVO::Vector2 &>())
        
        // Tuple-based overloads
        .def("add_agent", [](RVOSIM &self, const py::object &position) {
            return self.addAgent(sequence_to_vector(position));
        })
        .def("add_agent", [](RVOSIM &self, const py::object &position,
                               float neighborDist, std::size_t maxNeighbors,
                               float timeHorizon, float timeHorizonObst,
                               float radius, float maxSpeed) {
            return self.addAgent(sequence_to_vector(position), neighborDist, maxNeighbors,
                                 timeHorizon, timeHorizonObst, radius, maxSpeed);
        })
        .def("add_agent", [](RVOSIM &self, const py::object &position,
                               float neighborDist, std::size_t maxNeighbors,
                               float timeHorizon, float timeHorizonObst,
                               float radius, float maxSpeed,
                               const py::object &velocity) {
            return self.addAgent(sequence_to_vector(position), neighborDist, maxNeighbors,
                                 timeHorizon, timeHorizonObst, radius, maxSpeed,
                                 sequence_to_vector(velocity));
        })
        .def("add_obstacle", [](RVOSIM &self, const py::object &vertices) {
            return self.addObstacle(iterable_to_vector2_list(vertices));
        })
        .def("do_step", &RVOSIM::doStep)
        .def("process_obstacles", &RVOSIM::processObstacles)

        // Core scalars
        .def("get_global_time", &RVOSIM::getGlobalTime)
        .def("get_num_agents", &RVOSIM::getNumAgents)
        .def("get_num_obstacle_vertices", &RVOSIM::getNumObstacleVertices)

        // Getter methods
        .def("get_agent_agent_neighbor", &RVOSIM::getAgentAgentNeighbor)
        .def("get_agent_max_neighbors", &RVOSIM::getAgentMaxNeighbors)
        .def("get_agent_max_speed", &RVOSIM::getAgentMaxSpeed)
        .def("get_agent_neighbor_dist", &RVOSIM::getAgentNeighborDist)
        .def("get_agent_num_agent_neighbors", &RVOSIM::getAgentNumAgentNeighbors)
        .def("get_agent_num_obstacle_neighbors", &RVOSIM::getAgentNumObstacleNeighbors)
        .def("get_agent_num_orca_lines", &RVOSIM::getAgentNumORCALines)
        .def("get_agent_obstacle_neighbor", &RVOSIM::getAgentObstacleNeighbor)
        .def("get_agent_orca_line", [](const RVOSIM &self, std::size_t agent_no, std::size_t line_no) {
            const auto &L = self.getAgentORCALine(agent_no, line_no);
            return py::make_tuple(vector_to_tuple(L.direction), vector_to_tuple(L.point));
        })
        .def("get_agent_position", &RVOSIM::getAgentPosition)
        .def("get_agent_pref_velocity", &RVOSIM::getAgentPrefVelocity)
        .def("get_agent_radius", &RVOSIM::getAgentRadius)
        .def("get_agent_time_horizon", &RVOSIM::getAgentTimeHorizon)
        .def("get_agent_time_horizon_obst", &RVOSIM::getAgentTimeHorizonObst)
        .def("get_agent_velocity", &RVOSIM::getAgentVelocity)
        .def("get_obstacle_vertex", &RVOSIM::getObstacleVertex)
        .def("get_next_obstacle_vertex", &RVOSIM::getNextObstacleVertexNo)
        .def("get_prev_obstacle_vertex", &RVOSIM::getPrevObstacleVertexNo)

         // Visibility queries with tuple/list
         .def("query_visibility", [](const RVOSIM &self, const py::object &p1, const py::object &p2) {
            return self.queryVisibility(sequence_to_vector(p1), sequence_to_vector(p2));
        })
        .def("query_visibility", [](const RVOSIM &self, const py::object &p1, const py::object &p2, float radius) {
            return self.queryVisibility(sequence_to_vector(p1), sequence_to_vector(p2), radius);
        })

        // Setters and configuration
        .def("set_agent_defaults", [](RVOSIM &self, float neighborDist, std::size_t maxNeighbors,
                                       float timeHorizon, float timeHorizonObst,
                                       float radius, float maxSpeed) {
            self.setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed);
        })
        .def("set_agent_defaults", [](RVOSIM &self, float neighborDist, std::size_t maxNeighbors,
                                       float timeHorizon, float timeHorizonObst,
                                       float radius, float maxSpeed, const py::object &velocity) {
            self.setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed,
                                  sequence_to_vector(velocity));
        })
        .def("set_agent_max_neighbors", &RVOSIM::setAgentMaxNeighbors)
        .def("set_agent_max_speed", &RVOSIM::setAgentMaxSpeed)
        .def("set_agent_neighbor_dist", &RVOSIM::setAgentNeighborDist)
        .def("set_agent_position", [](RVOSIM &self, std::size_t agent_no, const py::object &p) {
            self.setAgentPosition(agent_no, sequence_to_vector(p));
        })
        .def("set_agent_pref_velocity", [](RVOSIM &self, std::size_t agent_no, const py::object &p) {
            self.setAgentPrefVelocity(agent_no, sequence_to_vector(p));
        })
        .def("set_agent_radius", &RVOSIM::setAgentRadius)
        .def("set_agent_time_horizon", &RVOSIM::setAgentTimeHorizon)
        .def("set_agent_time_horizon_obst", &RVOSIM::setAgentTimeHorizonObst)
        .def("set_agent_velocity", [](RVOSIM &self, std::size_t agent_no, const py::object &p) {
            self.setAgentVelocity(agent_no, sequence_to_vector(p));
        })
        .def("set_time_step", &RVOSIM::setTimeStep)
        .def("get_time_step", &RVOSIM::getTimeStep);
    }