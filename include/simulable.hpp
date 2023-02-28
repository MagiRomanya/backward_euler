#ifndef SIMULABLE_H_
#define SIMULABLE_H_
#include "integrator.hpp"

class Integrator;

class Simulable {
    public:
        unsigned int nDoF;
        unsigned int index;
        Integrator* integrator = nullptr;
        bool initialized = false;

        /*
         * Fill containers MUST fill the following containers
         * from integrator:
         *  - The mass matrix via the mass triplets
         *  - The equation matrix via the equation triplets
         *  - The positions and velocities x and v
         *  - The force and it's derivatives
        */
        virtual void fill_containers() {};

        virtual void update_state() {};

};

#endif // SIMULABLE_H_
