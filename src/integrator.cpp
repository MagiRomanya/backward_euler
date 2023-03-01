#include "integrator.hpp"
#include <iostream>

void Integrator::resize_containers() {
    f0.resize(nDoF);
    x.resize(nDoF);
    v.resize(nDoF);
    delta_v.resize(nDoF);

    mass.resize(nDoF, nDoF);
    df_dv.resize(nDoF, nDoF);
    df_dx.resize(nDoF, nDoF);

    clear_containers();
}

void Integrator::resize_containers(unsigned int newDoF) {
    nDoF = newDoF;
    resize_containers();
}

void Integrator::clear_containers() {
    f0.setZero();
    x.setZero();
    v.setZero();

    df_dx.setZero();
    df_dv.setZero();

    df_dx_triplets.clear();
    df_dv_triplets.clear();
    mass_triplets.clear();
    equation_matrix_triplets.clear();
}

void Integrator::add_simulable(Simulable* simulable){
    unsigned int index = this->nDoF;
    resize_containers(this->nDoF + simulable->nDoF);
    simulable->index = index;
    simulable->integrator = this;
    simulable->initialized = true;
    simulables.push_back(simulable);
}

void Integrator::integration_step() {
    clear_containers();

    if (simulables.size() == 0){
        std::cout << "ERROR::INTEGRATOR::INTEGRATION_STEP: No simulables added!" << std::endl;
        exit(-1);
    }

    // Fill containers
    for (int i = 0; i < simulables.size(); i++){
        Simulable* sim = simulables[i];
        sim->fill_containers();
    }

    implicit_euler();

    // update simulables
    for (int i = 0; i < simulables.size(); i++){
        simulables[i]->update_state();
    }
}

void Integrator::implicit_euler() {
    Eigen::SparseMatrix<double> equation_matrix(nDoF, nDoF);
    Eigen::VectorXd equation_vector;

    df_dx.setFromTriplets(df_dx_triplets.begin(), df_dx_triplets.end());
    df_dv.setFromTriplets(df_dv_triplets.begin(), df_dv_triplets.end());
    equation_matrix.setFromTriplets(equation_matrix_triplets.begin(), equation_matrix_triplets.end());

    equation_vector = h * (f0 + h * df_dx * v);

    // Gradient conjugate solving method class
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg;

    // Solving the system of equations
    cg.compute(equation_matrix);
    delta_v = cg.solve(equation_vector);

}
