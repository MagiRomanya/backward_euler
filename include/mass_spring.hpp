#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "object.h"
#include "parameter_list.hpp"
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
        /* Define Mass Spring with a single parameter for all the springs */
        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring);

        /* Define Mass Spring with 2 parameters: one for tension springs and another for bend springs */
        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring, double k_bend);

        /* Define Mass Spring with N paramters: a different paramter for each spring */
        MassSpring(Integrator* integrator, Object* obj, double node_mass,
                   const std::vector<double>& k_spring, const std::vector<double>& k_bend);

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

        // Variables when using 2 paramters
        ParameterList normalSpringParameters;
        ParameterList bendSpringParameters;

        // Variables when using N parameters
        bool vector_paramters = false;
        std::vector<ParameterList> parameters;
        unsigned int bend_offset = 0;
        unsigned int flex_counter = 0;
        unsigned int bend_counter = 0;
};

#endif // MASS_SPRING_H_
