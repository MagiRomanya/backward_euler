#include "spring_list.h"

// DEPRECATED for interaction vector in the system class

void Spring_list::add_spring_forces(System &s) const {
    /* Adds the force between particles joined with springs to the system */
    for (int i = 0; i < springs.size(); i++) {
        int p1 = springs[i].i;
        int p2 = springs[i].j;

        // Calculate the force of the string joining p1 and p2
        vec3 force = springs[i].force(s);
        if (!s.fixed[p1]){
            s.f0(3 * p1) += force.x();     // f0.x
            s.f0(3 * p1 + 1) += force.y(); // f0.y
            s.f0(3 * p1 + 2) += force.z(); // f0.z
        }
        // 3º Newton law: Equal and oposite reaction
        if (!s.fixed[p2]){
            s.f0(3 * p2) += -force.x();     // f0.x
            s.f0(3 * p2 + 1) += -force.y(); // f0.y
            s.f0(3 * p2 + 2) += -force.z(); // f0.z
        }
    }
}

void Spring_list::add_energy(System &s) const {
    /* Adds the potential energy of the spring to the total energy of the system */
    for (int i; i < springs.size(); i++){
        s.energy += springs[i].energy(s);
    }
}

void Spring_list::spring_derivatives_finite(System &s) const {
    /* Calculates the spring force derivateves using a finite step */
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
}

void Spring_list::add_spring_derivatives(System &s) {
    /* Adds the spring force derivatives to the system */
    for (int i = 0; i < springs.size(); i++) {
        springs[i].add_derivative(s);
    }
}
