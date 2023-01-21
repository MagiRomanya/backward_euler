#ifndef SPRING_H
#define SPRING_H

#include "interaction.h"
#include "system.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <math.h>
#include <ostream>
#include <vector>
#include <raylib.h>

// TODO: Convert the new_spring function to the constructor of the same class
class Spring : public Interaction {
    public:
        Spring() {}
        Spring(unsigned int pi, unsigned int pj, double sk, double sL0)
            : i(pi), j(pj), k(sk), L0(sL0) { }
        // The two particles bound by a spring
        unsigned int i, j;
        // The spring stiffness & the spring rest length
        double k, L0;

        vec3 force(const System &s) const ;
        double energy(const System &s) const ;
        Eigen::Matrix3d force_derivative(const System &s) const ;
        Eigen::Matrix3d force_derivative_finite(System &s) const ;

        void add_energy(System &s) const ; // to implement
        void add_force(System &s) const ; // To implement
        void add_derivative(System &s) const ;
        void apply(System &s);
        void render(System &s);
};

#endif
