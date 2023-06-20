#include <pybind11/eigen.h> // for sparse and dense matrices
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // for std::vector as argument

#include "pysimulation.hpp"
#include "spring_counter.hpp"

namespace py = pybind11;

PYBIND11_MODULE(symulathon, m) {
    m.doc() = "A simple simulation backend.";

    m.def("cout_spring", &count_spring, "Returns the number of normal and bend springs of a mesh in that order");

    // Main simulation interface
    py::class_<PySimulation>(m, "Simulation")
        .def(py::init<double, double, bool>(),
             py::arg("k"),
             py::arg("k_bend"),
             py::arg("graphics") = false)
        .def(py::init<std::vector<double>, std::vector<double>, bool>(),
             py::arg("k"),
             py::arg("k_bend"),
             py::arg("graphics") = false)
        .def("fill_containers", &PySimulation::fill_containers)
        .def("set_state", &PySimulation::set_state)
        .def("getEquationMatrix", &PySimulation::getEquationMatrix)
        .def("getEquationVector", &PySimulation::getEquationVector)
        .def("getForce", &PySimulation::getForce)
        .def("getPosition", &PySimulation::getPosition)
        .def("getVelocity", &PySimulation::getVelocity)
        .def("getMassMatrix", &PySimulation::getMassMatrix)
        .def("getParameterJacobian", &PySimulation::getParameterJacobian)
        .def("getForcePositionJacobian", &PySimulation::getForcePositionJacobian)
        .def("getDoF", &PySimulation::getDoF)
        .def("getTimeStep", &PySimulation::getTimeStep)
        .def("render_state", &PySimulation::render_state)
        ;
}
