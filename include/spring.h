#ifndef SPRING_H
#define SPRING_H

#include <math.h>
#include <ostream>
#include <vector>

#include "interaction.h"
#include "vec3.h"
#include "particle_system.hpp"
#include "integrator.hpp"

class Spring : public Interaction {
    public:
        Spring() {}
        Spring(unsigned int pi, unsigned int pj, double sk, double sL0)
            : i(pi), j(pj), k(sk), L0(sL0) { }

        // The two particles bound by a spring (in ParticleSystem coordinates)
        unsigned int i, j;
        // The spring stiffness & the spring rest length
        double k, L0;

        void apply(Integrator* integrator) override;

    private:
        // Coordinates and lenth for a frame
        float L;
        vec3 x1, x2;

        // Indices in the integrator
        unsigned int itg_i, itg_j;

        void get_state(ParticleSystem &sys);

        vec3 force(const Integrator &itg) const ;
        double energy(const Integrator &itg) const ;

        Eigen::Matrix3d force_derivative(const Integrator &itg) const ;
        Eigen::Matrix3d force_derivative_finite(Integrator &itg) const ;

        void add_force(Integrator &itg) const ;
        void add_derivative(Integrator &itg) const ;
};

#endif
