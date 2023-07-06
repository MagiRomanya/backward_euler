#ifndef NON_PHYSICAL_H_
#define NON_PHYSICAL_H_

#include "simulable.hpp"
#include "vec3.hpp"
#include "object.h"

class NonPhysical : public Simulable {
    public:
        NonPhysical(Object& obj);

        /* Fill contaniers does nothing */
        inline void fill_containers() override {}

        /* Set state does nothing */
        void set_state() override {}

        /* Update state will dictate how the object moves */
        void update_state() override;

    private:
        Object& render_object;
        vec3 position;
        vec3 velocity = vec3(0, 0, 1);
};

#endif // NON_PHYSICAL_H_
