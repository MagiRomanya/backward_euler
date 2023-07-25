#ifndef NON_PHYSICAL_H_
#define NON_PHYSICAL_H_

#include "integrator.hpp"
#include "simulable.hpp"
#include "vec3.hpp"
#include "object.h"

class NonPhysical : public Simulable {
    public:
        NonPhysical(Object& obj);

        NonPhysical(Object& obj, Integrator& itg, vec3 (*trajectory_fun)(float));

        inline void fill_containers() override;

        void set_state() override;

        void update_state() override;

    private:
        Object& render_object;
        vec3 position;
        vec3 velocity = vec3(0, 0, 1);
        float time = 0;
        vec3 (*trajectory_function)(float);
};

#endif // NON_PHYSICAL_H_
