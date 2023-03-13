#ifndef INTERACTION_H
#define INTERACTION_H

#include "integrator.hpp"

// class System;
class ParticleSystem;

class Interaction{
/* Abstract class which represents any interaction that can apply force to a particle
 * Examples: springs, gravity, friction, etc  */
    public:
        // virtual void apply(System &s) {}
        virtual void apply(Integrator &itg, ParticleSystem* sys) = 0;
};


#endif // INTERACTION_H
