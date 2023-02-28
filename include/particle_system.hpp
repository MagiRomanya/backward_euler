#ifndef PARTICLE_SYSTEM_H_
#define PARTICLE_SYSTEM_H_

#include <vector>

#include "vec3.h"
#include "interaction.h"
#include "simulable.hpp"

class ParticleSystem : public Simulable {

    public:
        std::vector<Interaction*> interactions;

        inline void fix_particle(int p){
            // Make a particle fixed: this particle is not affected by any interaction and will not move
            fixed[p] = true;
            fixed_particles.push_back(p);
        }

        void resize_containers(unsigned int nDoF);

        void fill_containers() override;

        void update_state() override;

        void set_particle_position(int index, vec3& pos);

        vec3 get_particle_position(int index);


    private:
        Eigen::VectorXd x;
        Eigen::VectorXd v;

        // Fixed particles
        std::vector<bool> fixed;
        std::vector<int> fixed_particles;
};

#endif // PARTICLE_SYSTEM_H_
