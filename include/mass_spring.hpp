#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "object.h"
#include "particle_system.hpp"
#include "spring.h"

enum COORDINATE_SYSTEM {
LOCAL,
WORLD,
};

class MassSpring : public ParticleSystem {
    public:

        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring);

        void fill_containers() override;

        void update_state() override;

    private:
        void update_mesh();

        void positions_to_local();

        void positions_to_world();

        void load_from_mesh(Object* obj, double node_mass, double k_spring);

        void add_spring(unsigned int i1, unsigned int i2, double K, double L);

        COORDINATE_SYSTEM current_coordinate_system = LOCAL;

        SimpleMesh* mesh = nullptr;
        Object* obj = nullptr;

        double k_spring;
        double k_bend;
};

#endif // MASS_SPRING_H_
