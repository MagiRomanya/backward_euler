#ifndef SIMULABLE_H_
#define SIMULABLE_H_

class Integrator;

/* Abstract class which represents a simulable object. The integrator will
 * ask a simulable to fill the containers, then it will do the integration and in the end
 * it will give the simulables the new coordinates back so that they can manage how to update
 * their state. */
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
         *  - The force and it's derivatives (also the equation triplets)
        */
        virtual void fill_containers() = 0;

        virtual void update_state() = 0;

};

#endif // SIMULABLE_H_
