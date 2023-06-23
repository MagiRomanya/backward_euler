#ifndef TWOBODY_H_
#define TWOBODY_H_

#include "interaction.h"
#include "integrator.hpp"
#include "particle_system.hpp"
#include "vec3.h"

class TwoBodyInteraction : public Interaction {
    public:
        TwoBodyInteraction(unsigned int pi, unsigned int pj);

        ~TwoBodyInteraction() {}

        void apply(Integrator &itg, ParticleSystem* sys) override;

    protected:
        // Particle index wrt ParticleSystem
        unsigned int p1, p2;
        // Indices wrt Integrator
        unsigned int itg_p1, itg_p2;

        // Relative distance
        double L;
        // Positions
        double x1, y1, z1;
        double x2, y2, z2;

        // Velocities
        double vx1, vy1, vz1;
        double vx2, vy2, vz2;

        // Parameter indexs
        std::vector<size_t> parameter_indices;

        /* Fills L, x1 and x2 */
        void get_state(ParticleSystem* sys);

        /* Fill the containers in the integrator */
        void add_force(Integrator &itg, ParticleSystem* sys);

        void add_force_derivatives(Integrator &itg, ParticleSystem* sys);

        void add_parameters_derivative(Integrator &itg, ParticleSystem *sys);

        /* Unique functions of the interaction */
        virtual double energy() const = 0;

        virtual vec3 force() const = 0;

        virtual Eigen::Matrix3d force_position_derivative() const = 0;

        virtual Eigen::Matrix3d force_velocity_derivative() const = 0;

        virtual Eigen::MatrixXd force_parameters_derivative() const = 0;
};

#endif // TWOBODY_H_
