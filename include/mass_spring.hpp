#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "object.h"
#include "particle_system.hpp"
#include "spring.h"
#include "spring_test.hpp"

enum COORDINATE_SYSTEM {
LOCAL,
WORLD,
};

enum SPRING_TYPE {
FLEX,
BEND,
};

class MassSpring : public ParticleSystem {
    public:
        MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring);

        void fill_containers() override;

        void update_state() override;

        inline double get_k() { return k_flex; }
        inline void set_k(double k) { k_flex = k; }

    private:
        void update_mesh();

        void positions_to_local();

        void positions_to_world();

        void load_from_mesh(Object* obj, double node_mass);

        void add_spring(unsigned int i1, unsigned int i2, SPRING_TYPE type, double L);

        COORDINATE_SYSTEM current_coordinate_system = LOCAL;

        SimpleMesh* mesh = nullptr;
        Object* obj = nullptr;

        double k_flex;
        double k_bend;

        std::list<double> lengths;

        vec3 gravity_vec;
};

#endif // MASS_SPRING_H_
