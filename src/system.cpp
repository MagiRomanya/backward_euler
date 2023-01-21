#include "system.h"

void System::update_vel_and_pos() {
    /* Update of the position & velocity vectors */
    for (int i=0; i < num; i++) {
        for (int j=0; j <3; j++) {
            v[3*i+j] += delta_v[3*i+j];
            x[3*i+j] += h * v[3*i+j];
        }
    }
}

void System::backward_euler_sparse() {
    /* Backward euler ODE solver.
     * Uses sparse data structures.
     * Uses conjugate gradient to solve the system of equations */

    // equation_matrix * x = equation_vector (x = delta_v)
    Eigen::SparseMatrix<double> equation_matrix(3*num, 3*num);
    Eigen::VectorXd equation_vector;

    df_dx_s.setFromTriplets(df_dx_triplets.begin(), df_dx_triplets.end());
    equation_matrix.setFromTriplets(equation_matrix_triplets.begin(), equation_matrix_triplets.end());

    equation_vector = h * (f0 + h * df_dx_s * v);

    // Gradient conjugate solving method class
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> cg;
    // Solving the system of equations
    cg.compute(equation_matrix);
    delta_v = cg.solve(equation_vector);
}

void System::begin_equation_matrix() {
    // First we should delete all of the memory related to the last frame
    // REVIEW: there has to be a better way to do this
    this->df_dv_triplets.clear();
    this->df_dx_triplets.clear();
    this->equation_matrix_triplets.clear();
    // We should reserve memory heare in the triplets containers.
    // There are four 3*3 derivative matrix per spring:
    // dfa/dxb = dfb/dxb = - dfa/dxb = - dfb/dxa
    equation_matrix_triplets.reserve(4*3*3*interactions.size());
    df_dx_triplets.reserve(4*3*3*interactions.size());

    // The formula for our equation matrix is the following:
    // equation_matrix = Mass_s - h * df_dv_s - h * h * df_dx_s;

    // In this function we add the mass
    const double mass = this->Mass_s.coeff(0,0); // assume all to have the same mass
    for (int i=0; i < 3*num; i++){
        equation_matrix_triplets.push_back(Eigen::Triplet<double>(i,i, mass));
    }
}

void System::calculate_kinetic_energy() {
    /* Calculated the kinetic energy of a particle */
    for (int i = 0; i < num; i++){
        vec3 vel = particle_velocity(i);
        energy += 0.5 * Mass(i,i) * vel.length2();
    }
}

void System::update() {
    /* Performs a step in the simulation */
    // Resets some values
    f0.setZero();
    df_dv_s.setZero();
    df_dx_s.setZero();
    begin_equation_matrix();
    energy = 0;

    // Calculates forces, derivatives and energies
    calculate_kinetic_energy();
    for (int i=0; i < interactions.size(); i++){
        interactions[i]->apply(*this);
    }

    // Add gravity
    double gravity = 1 * Mass(0, 0); // assuming all equal masses
    for (int i = 0; i < num ; i += 1) {
        if (!fixed[i])
            f0[3*i + 2] += gravity; // z direction
    }

    // Calculate the next velocities and positions
    backward_euler_sparse();
    update_vel_and_pos();
}

System::~System(){
    // The interactions were created in the heap
    for (int i = 0; i < interactions.size(); i++){
        delete interactions[i];
    }
}

void System::render(){
    int offset = 100;
    float radius = 5;
    // Drawing nodes
    for (int i=0; i < 3 * num; i+=3){
        DrawCircle(offset + x[i+1], offset + x[i+2], radius,
                   (Color){0,0,0,255});
        DrawCircle(x[i + 1] + 100, x[i] + 100, radius,
                   (Color){255, 0, 0, 255}); // 3d pesrspective
    }
    // Draw interactions
    for (int i=0; i < interactions.size(); i++){
        interactions[i]->render(*this);
    }
}

void System::update_dimensions(int new_particle_number){
    /* Call this function whenever we change the number of particles
     * in the system */
    num = new_particle_number;

     // Vectors
     x.resize(3*num);
     v.resize(3*num);
     f0.resize(3*num);
     delta_v.resize(3*num);
     fixed.resize(num, false);

     // Matrices
     Mass.resize(3*num, 3*num);
     Mass_s.resize(3*num, 3*num);
     df_dv_s.resize(3*num, 3*num);
     df_dx_s.resize(3*num, 3*num);

     // Update the values
     x.setZero();
     v.setZero();
     f0.setZero();
     delta_v.setZero();

     Mass.setIdentity();
     Mass_s.setIdentity();
}
