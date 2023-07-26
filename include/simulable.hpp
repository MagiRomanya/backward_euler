#ifndef SIMULABLE_H_
#define SIMULABLE_H_

#include <Eigen/Eigen>

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

        void initialize(Integrator* integrator, unsigned int index)
        {
            initialized = true;
            this->index = index;
            this->integrator = integrator;
        }

        /*
         * Fill containers MUST fill the following containers
         * from integrator:
         *  - The mass matrix via the mass triplets
         *  - The equation matrix via the equation triplets
         *  - The positions and velocities x and v
         *  - The force and it's derivatives (also the equation triplets)
        */
        virtual void fill_containers() = 0;

        /*
         * Update state modifies the simulable's state from the
         * integrator's delta v
         */
        virtual void update_state() = 0;

        /*
         * Set state modifies the simulable's state setting the positions
         * and velocities from the ones stored in the integrator.
         */
        virtual void set_state() = 0;

        /*
        ** Computes the derivatives of the initial state with respect to the
        ** differentiable parameters.
        ** The default behaviour is to set all the derivatives to zero
         */
        virtual void get_initial_state_jacobian(Eigen::SparseMatrix<double>& dx0dp, Eigen::SparseMatrix<double>& dv0dp);

};

#endif // SIMULABLE_H_
