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
    df_dp.resize(nDoF, nParameters);

    clear_containers();
}

void Integrator::resize_containers(unsigned int newDoF, unsigned int newNConstraints, unsigned int newNParameters) {
    nDoF = newDoF;
    nConstraints = newNConstraints;
    nParameters = newNParameters;
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
    unsigned int p_index = this->nParameters;
    resize_containers(this->nDoF + simulable->nDoF, nConstraints,
                      nParameters + simulable->nParameters);
    simulable->initialize(this, index, p_index);
    simulables.push_back(simulable);
    update_constraint_indices();
}

void Integrator::add_constraint(Constraint* constraint){
    unsigned int index = this->nConstraints;
    resize_containers(nDoF, nConstraints + constraint->nConstraints, nParameters);
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
        std::cout << "ERROR::INTEGRATOR::INTEGRATION_STEP: No simulables added!" << std::endl;
        exit(-1);
    }

    // Fill containers
    for (int i = 0; i < simulables.size(); i++) {
        Simulable* sim = simulables[i];
        sim->fill_containers();
    }
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
    Eigen::VectorXd equation_vector;
    df_dx.setFromTriplets(df_dx_triplets.begin(), df_dx_triplets.end());
    df_dv.setFromTriplets(df_dv_triplets.begin(), df_dv_triplets.end());
    equation_vector = h * (f0 + h * df_dx * v);
    return equation_vector;
}
