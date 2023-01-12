#include "system.h"
#include "clock.h"

void System::update_vel_and_pos() {
    for (int i=0; i < num; i++) {
        for (int j=0; j <3; j++) {
            v[3*i+j] += delta_v[3*i+j];
            x[3*i+j] += h * v[3*i+j];
        }
    }
}

void System::backward_euler_sparse() {
    // We calculate the system of equations we want to solve
    // equation_matrix * x = equation_vector
    Eigen::SparseMatrix<double> equation_matrix(3*num, 3*num);
    Eigen::VectorXd equation_vector;

    df_dx_s.setFromTriplets(df_dx_triplets.begin(), df_dx_triplets.end());
    equation_matrix.setFromTriplets(equation_matrix_triplets.begin(), equation_matrix_triplets.end());

    equation_vector = h * (f0 + h * df_dx_s * v);
    // std::cout << equation_vector << std::endl;
    // std::cout << std::endl;

    // Gradient conjugate solving method class
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg;
    // Solving the system of equations
    cg.compute(equation_matrix);
    delta_v = cg.solve(equation_vector);
}

void System::begin_equation_matrix() {
    // First we should delete all of the memory related to the last frame (there has to be a better way to do this)
    this->df_dv_triplets.clear();
    this->df_dx_triplets.clear();
    this->equation_matrix_triplets.clear();
    // We should reserve memory heare in the triplets containers. I have no idea how to predict the amount of items that we will need
    const int N = this->num; // number of particles
    equation_matrix_triplets.reserve(4*3*3*N + 3*N);
    df_dx_triplets.reserve(4*3*3*N);

    // The formula for our equation matrix is the following:
    // equation_matrix = Mass_s - h * df_dv_s - h * h * df_dx_s;

    // In this function we add the mass
    const double mass = this->Mass_s.coeff(0,0); // assume all to have the same mass
    for (int i=0; i < 3*N; i++){
        equation_matrix_triplets.push_back(Eigen::Triplet<double>(i,i, mass));
    }
}
