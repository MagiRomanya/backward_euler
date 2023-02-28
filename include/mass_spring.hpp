#ifndef MASS_SPRING_H_
#define MASS_SPRING_H_

#include "mesh.h"
#include "particle_system.hpp"
#include "spring.h"

class MassSpring : public ParticleSystem {
    public:
        MassSpring(SimpleMesh* mesh, double k_spring);
        ~MassSpring();

    private:
        void load_from_mesh(SimpleMesh* mesh, double k_spring);
        SimpleMesh* mesh;
};

#endif // MASS_SPRING_H_
