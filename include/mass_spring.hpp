#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "mesh.h"
#include "particle_system.hpp"
#include "spring.h"

class MassSpring : public ParticleSystem {
    public:
        MassSpring(Integrator* integrator, SimpleMesh* mesh, double node_mass, double k_spring);
        ~MassSpring();

        void update_state() override;

        void update_mesh();

    private:
        void load_from_mesh(SimpleMesh* mesh, double node_mass, double k_spring);
        SimpleMesh* mesh;
};

#endif // MASS_SPRING_H_
