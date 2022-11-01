#include "spring_list.h"

void Spring_list::add_spring_forces(System &s) const {
    for (int i = 0; i < springs.size(); i++) {
        int p1 = springs[i].i;
        int p2 = springs[i].j;

        // Calculate the force of the string joining p1 and p2
        vec3 force = springs[i].force(s);
        s.f0(3 * p1) += force.x();     // f0.x
        s.f0(3 * p1 + 1) += force.y(); // f0.y
        s.f0(3 * p1 + 2) += force.z(); // f0.z

        // 3º Newton law: Equal and oposite reaction
        s.f0(3 * p2) += -force.x();     // f0.x
        s.f0(3 * p2 + 1) += -force.y(); // f0.y
        s.f0(3 * p2 + 2) += -force.z(); // f0.z
    }
}

void Spring_list::spring_derivatives_finite(System &s) const {
    Eigen::MatrixXd df_dx(3*s.num, 3*s.num);
    df_dx.setZero();
    double epsilon = s.h;
    for (int i=0; i < springs.size(); i++){
        int p1 = springs[i].i;
        int p2 = springs[i].j;
        vec3 force = springs[i].force(s); // f(x)
        // Each spring afects a 3x3 matrix inside the huge df_dx matrix
        for (int a=0; a < 3; a++){
            for (int b=0; b < 3; b++){
                s.x(3*p1 +b) += epsilon;
                vec3 new_force = springs[i].force(s); // f(x + epsilon)
                df_dx(3*p1 +a, 3*p1 +b) += 1/epsilon * (new_force[a] - force[a]);
                df_dx(3*p2 +a, 3*p1 +b) += -1/epsilon * (new_force[a] - force[a]);
                s.x(3*p1 +b) -= epsilon;

                s.x(3*p2 +b) += epsilon;
                new_force = springs[i].force(s); // f(x + epsilon)
                df_dx(3*p1 +a, 3*p2 +b) += 1/epsilon * (new_force[a] - force[a]);
                df_dx(3*p2 +a, 3*p2 +b) += -1/epsilon * (new_force[a] - force[a]);
                s.x(3*p2 +b) -= epsilon;
            }
        }
    }
    s.df_dx = df_dx;
}

void Spring_list::add_spring_derivatives(System &s) const {
    for (int i = 0; i < springs.size(); i++) {
        // Calculates the force derivative matrix of a given string
        // and adds the result to the global df/dx derivative matrix
        int p1 = springs[i].i;
        int p2 = springs[i].j;
        Eigen::Matrix3d df_dx = springs[i].force_derivative(s);
        // Add the derivative to the big matrix componentwise
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                // df1_dx1
                s.df_dx(3 * p1 + j, 3 * p1 + k) += df_dx(j, k);
                s.df_dx_s.coeffRef(3 * p1 + j, 3 * p1 + k) += df_dx(j, k);
                // df2_dx1
                s.df_dx(3 * p1 + j, 3 * p2 + k) -= df_dx(j, k);
                s.df_dx_s.coeffRef(3 * p1 + j, 3 * p2 + k) -= df_dx(j, k);
                // df1_dx2
                s.df_dx(3 * p2 + j, 3 * p1 + k) -= df_dx(j, k);
                s.df_dx_s.coeffRef(3 * p2 + j, 3 * p1 + k) -= df_dx(j, k);
                // df2_dx2
                s.df_dx(3 * p2 + j, 3 * p2 + k) += df_dx(j, k);
                s.df_dx_s.coeffRef(3 * p2 + j, 3 * p2 + k) += df_dx(j, k);
            }
        }
    }
}
