#include "integrator.hpp"
#include <iostream>

void Integrator::resize_containers() {
    f0.resize(nDoF);
    x.resize(nDoF);
    v.resize(nDoF);
    delta_v.resize(nDoF);
    constraint_value.resize(nConstraints);

    mass.resize(nDoF, nDoF);
    df_dv.resize(nDoF, nDoF);
    df_dx.resize(nDoF, nDoF);
    df_dp.resize(nDoF, diff_manager.get_size());

    clear_containers();
}

void Integrator::resize_containers(unsigned int newDoF, unsigned int newNConstraints) {
    nDoF = newDoF;
    nConstraints = newNConstraints;
    resize_containers();
}

void Integrator::clear_containers() {
    x.setZero();
    v.setZero();
    f0.setZero();
    delta_v.setZero();
    constraint_value.setZero();

    df_dx.setZero();
    df_dv.setZero();
    mass.setZero();

    df_dx_triplets.clear();
    df_dv_triplets.clear();
    mass_triplets.clear();
    equation_matrix_triplets.clear();
    constraint_jacobian_triplets.clear();

    df_dp.setZero(nDoF, diff_manager.get_size());
}

void Integrator::add_simulable(Simulable* simulable){
    unsigned int index = this->nDoF;
    resize_containers(this->nDoF + simulable->nDoF, nConstraints);
    simulable->initialize(this, index);
    simulables.push_back(simulable);
    update_constraint_indices();
}

void Integrator::add_constraint(Constraint* constraint){
    unsigned int index = this->nConstraints;
    resize_containers(nDoF, nConstraints + constraint->nConstraints);
    constraint->index = index;
    constraint->itg = this;
    constraints.push_back(constraint);
}

void Integrator::integration_step() {
    fill_containers();

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

    if (nConstraints) {
        generate_equation_with_constraints(equation_matrix, equation_vector);
    }
    else {
        equation_matrix.setFromTriplets(equation_matrix_triplets.begin(), equation_matrix_triplets.end());

        equation_vector = h * (f0 + h * df_dx * v);
    }

    // Gradient conjugate solving method class
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg;

    // Solving the system of equations
    cg.compute(equation_matrix);
    delta_v = cg.solve(equation_vector);

}

void Integrator::fill_containers(){
    clear_containers();

    if (simulables.size() == 0){
        std::cerr << "ERROR::INTEGRATOR::INTEGRATION_STEP: No simulables added!" << std::endl;
        exit(-1);
    }

    // Fill containers
    for (int i = 0; i < simulables.size(); i++) {
        Simulable* sim = simulables[i];
        sim->fill_containers();
    }

    // Create sparse matrices from triplets
    mass.setFromTriplets(mass_triplets.begin(), mass_triplets.end());
    df_dx.setFromTriplets(df_dx_triplets.begin(), df_dx_triplets.end());
    df_dv.setFromTriplets(df_dv_triplets.begin(), df_dv_triplets.end());
}

void Integrator::generate_equation_with_constraints(Eigen::SparseMatrix<double>& equation_matrix, Eigen::VectorXd equation_vector) {
    /* Constructs the equation matrix and equation vector assuming the  */
    Eigen::VectorXd b = h * (f0 + h * df_dx * v);

    for (size_t i = 0; i < constraint_jacobian_triplets.size(); i++) {
        tri& t = constraint_jacobian_triplets[i];
        equation_matrix_triplets.push_back(t);
    }

    equation_matrix.resize(nDoF + nConstraints, nDoF + nConstraints);
    equation_matrix.setFromTriplets(equation_matrix_triplets.begin(), equation_matrix_triplets.end());

    equation_vector.resize(equation_vector.size() + constraint_value.size());
    Eigen::VectorXd constraint_term = - 1.0 / h * constraint_value;
    equation_vector << equation_vector, constraint_term;
}

void Integrator::update_constraint_indices() {
    /* Sets the index for the constraint jacobian matrix */
    unsigned int ind = nDoF;
    for (size_t i = 0; i < constraints.size(); i++) {
        ind += constraints[i]->nConstraints;
        constraints[i]->jindex = ind;
    }
}

void Integrator::reciveDeltaV(Eigen::VectorXd delta_v) {
    // update simulables
    this->delta_v = delta_v;
    for (int i = 0; i < simulables.size(); i++){
        simulables[i]->update_state();
    }
}

Eigen::SparseMatrix<double> Integrator::getEquationMatrix() {
    Eigen::SparseMatrix<double> equation_matrix(nDoF, nDoF);
    equation_matrix.setFromTriplets(equation_matrix_triplets.begin(), equation_matrix_triplets.end());
    return equation_matrix;
}


Eigen::VectorXd Integrator::getEquationVector() {
    // df_dx.setFromTriplets(df_dx_triplets.begin(), df_dx_triplets.end());
    // df_dv.setFromTriplets(df_dv_triplets.begin(), df_dv_triplets.end());
    return h * (f0 + h * df_dx * v);
}

Eigen::VectorXd Integrator::getForceVector() {
    return f0;
}

void Integrator::clear_simulables() {
    simulables.clear();
    constraints.clear();
    nDoF = 0;
    diff_manager.clear();
    clear_containers();
}

void Integrator::set_state(Eigen::VectorXd xi, Eigen::VectorXd vi) {
    if (xi.size() != vi.size() && xi.size() != x.size()) {
        std::cerr << "WARNING::INTEGRATOR::SET_STATE: new state dimensions do not match" << std::endl;
        return;
    }
    x = xi;
    v = vi;
    for (int i = 0; i < simulables.size(); i++) {
        simulables[i]->set_state();
    }
}
