#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "object.h"
#include "particle_system.hpp"
#include "spring.h"

class MassSpring : public ParticleSystem {
    public:

        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring);

        ~MassSpring();

        void fill_containers() override;

        void update_state() override;

        void update_mesh();

    private:
        void positions_to_local();

        void positions_to_world();

        void load_from_mesh(Object* obj, double node_mass, double k_spring);

        SimpleMesh* mesh = nullptr;
        Object* obj = nullptr;
};

#endif // MASS_SPRING_H_
