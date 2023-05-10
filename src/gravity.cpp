#include "gravity.hpp"

void Gravity::apply(Integrator &itg, ParticleSystem *sys){
    if (!all){
        if (!sys->is_fixed(index)){
            itg.f0(sys->index + 3*index)     += gravity->x();
            itg.f0(sys->index + 3*index + 1) += gravity->y();
            itg.f0(sys->index + 3*index + 2) += gravity->z();
        }
        return;
    }

    for (size_t i = 0; i < sys->nDoF; i += 3){
        if (!sys->is_fixed(i/3)){
            itg.f0(sys->index + i )   += gravity->x();
            itg.f0(sys->index + i+1 ) += gravity->y();
            itg.f0(sys->index + i+2 ) += gravity->z();
        }
    }

}
