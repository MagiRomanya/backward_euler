#ifndef SIMULABLE_H_
#define SIMULABLE_H_
#include "integrator.hpp"

class Simulable {
    public:
        unsigned int nDoF;
        unsigned int index;
        Integrator* integrator = nullptr;
        bool initialized = false;

        virtual void fill_containers();

        virtual void update_state();

};

#endif // SIMULABLE_H_
