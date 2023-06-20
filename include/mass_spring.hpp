#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "object.h"
#include "particle_system.hpp"
#include "simulable.hpp"
#include "spring.h"
#include "spring_test.hpp"
#include <iterator>

enum SPRING_TYPE {
FLEX,
BEND,
};

class MassSpring : public ParticleSystem {
    public:
        /* Define Mass Spring with a single parameter for all the springs */
        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring);

        /* Define Mass Spring with 2 parameters: one for tension springs and another for bend springs */
        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring, double k_bend);

        /* Define Mass Spring with N paramters: a different paramter for each spring */
        MassSpring(Integrator* integrator, Object* obj, double node_mass,
                   std::vector<double> k_spring, std::vector<double> k_bend);

        void update_state() override;

        void set_state() override;

    private:
        void update_mesh();

        void positions_to_local();

        void positions_to_world();

        void load_from_mesh(Integrator* itg, Object* obj, double node_mass);

        void add_spring(Integrator* itg, unsigned int i1, unsigned int i2, SPRING_TYPE type, double L);

        Object* obj = nullptr;

        vec3 gravity_vec;

        ParameterList normalSpringParameters;
        ParameterList bendSpringParameters;

        bool vector_paramters = false;
        std::vector<double>::iterator k_springs;
        std::vector<double>::iterator k_bends;
};

#endif // MASS_SPRING_H_
