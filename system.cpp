#include "system.h"


void System::update_vel_and_pos() {
    for (int i=0; i < num; i++) {
        if (!fixed[i]) {
            for (int j=0; j <3; j++) {
                v[3*i+j] += delta_v[3*i+j];
                x[3*i+j] += h * v[3*i+j];
            }
        }
    }
    // v += delta_v;
    // x += h * v;
}

void System::backward_euler() {
    // We calculate the system of equations we want to solve
    // equation_matrix * x = equation_vector
    Eigen::MatrixXd equation_matrix = Mass - h * df_dv - h * h * df_dx;
    Eigen::VectorXd equation_vector = h * (f0 + h * df_dx * v);
    // Gradient conjugate solving method class
    Eigen::ConjugateGradient<Eigen::MatrixXd> cg;
    // Solving the system of equations
    cg.compute(equation_matrix);
    delta_v = cg.solve(equation_vector);
}

// TODO: Do the profiling of this function to identify the bottleneck
void System::backward_euler_sparse() {
    // We calculate the system of equations we want to solve
    // equation_matrix * x = equation_vector
    Eigen::SparseMatrix<double> equation_matrix = Mass - h * df_dv_s - h * h * df_dx_s;
    Eigen::VectorXd equation_vector = h * (f0 + h * df_dx_s * v);
    // Gradient conjugate solving method class
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg;
    // Solving the system of equations
    cg.compute(equation_matrix);
    delta_v = cg.solve(equation_vector);
}
