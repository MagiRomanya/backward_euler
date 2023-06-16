#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "object.h"
#include "particle_system.hpp"
#include "simulable.hpp"
#include "spring.h"
#include "spring_test.hpp"

enum SPRING_TYPE {
FLEX,
BEND,
};

class MassSpring : public ParticleSystem {
    public:
        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring);

        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring, double k_bend);

        // MassSpring(Integrator* integrator, double node_mass, double k_spring);

        void update_state() override;

        void set_state() override;

    private:
        void update_mesh();

        void positions_to_local();

        void positions_to_world();

        void load_from_mesh(Object* obj, double node_mass, Integrator* itg);

        void add_spring(unsigned int i1, unsigned int i2, SPRING_TYPE type, double L);

        Object* obj = nullptr;

        vec3 gravity_vec;

        ParameterList normalSpringParameters;
        ParameterList bendSpringParameters;
};

#endif // MASS_SPRING_H_
