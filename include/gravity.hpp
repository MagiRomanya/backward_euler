#ifndef GRAVITY_H
#define GRAVITY_H

#include "interaction.h"
#include "particle_system.hpp"
#include "integrator.hpp"
#include "vec3.h"

class Gravity : public Interaction {
    public:
        // Applies gravity to all the particles in the particle system
        Gravity(vec3 gravity) :  gravity(gravity) { all = true; };

        Gravity(unsigned int index, vec3 gravity) :  index(index), gravity(gravity) { all = false; };

        void apply(Integrator &itg, ParticleSystem *sys) override;

    private:
        bool all;
        vec3 gravity;
        unsigned int index;
};


#endif // GRAVITY_H
