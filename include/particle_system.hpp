#ifndef PARTICLE_SYSTEM_H_
#define PARTICLE_SYSTEM_H_

#include <system_error>
#include <vector>

#include "vec3.hpp"
#include "interaction.hpp"
#include "simulable.hpp"

class ParticleSystem : public Simulable {

    public:
        inline void fix_particle(int p){
            // Make a particle fixed: this particle is not affected by any interaction and will not move
            if (p < n_particles) {
                fixed[p] = true;
                fixed_particles.push_back(p);
            }
            else
                std::cerr << "WARNING::PARTICLESYSTEM::FIX_PARTICLE: Particle " << p << ", is outside bounds and has not been fixed." << std::endl;
        }

        ~ParticleSystem();

        void resize_containers(unsigned int nDoF);

        void fill_containers() override;

        void update_state() override;

        void set_state() override;

        void set_particle_position(int index, vec3& pos);

        vec3 get_particle_position(int index);

        vec3 get_particle_velocity(int index);

        inline unsigned int get_n_particles() const { return n_particles; }

        inline bool is_fixed(int index) const { return fixed[index]; }

        inline std::vector<int> get_fixed_particles() {return fixed_particles; }

        inline void add_interaction(Interaction *interaction) {
            interactions.push_back(interaction);
        }

    protected:
        std::vector<Interaction*> interactions;

        std::vector<Interaction*> class_allocated_interactions;

        Eigen::VectorXd x;
        Eigen::VectorXd v;

        std::vector<double> mass;

        unsigned int n_particles;

        // Fixed particles
        std::vector<bool> fixed;
        std::vector<int> fixed_particles;
};

#endif // PARTICLE_SYSTEM_H_
