#include "pysimulation.hpp"

PySimulation::PySimulation(std::vector<double> parameters) {
    integrator = std::make_unique<Integrator>(TimeStep);

    // Create simulable
    setUpCloth();
    double k = parameters[0];
    double k_bend = parameters[1];
    mass_spring = std::make_unique<MassSpring>(integrator.get(), &cloth, NODE_MASS, k, k_bend);

    // Fix corners
    const int index = M * (N - 1);
    mass_spring->fix_particle(0);
    mass_spring->fix_particle(index);
    integrator->fill_containers();
}

void PySimulation::setUpCloth() {
    const double step = 0.5;
    CreateGrid(mesh, N, M, step);
    cloth = Object(&mesh);

    // Handle model matrix
    cloth.translation = glm::vec3(-0.5 * N, 0.0f, -4.0f*N);
    cloth.scaling = glm::vec3(3.0);
    cloth.updateModelMatrix();
}
