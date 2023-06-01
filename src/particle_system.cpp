#include "particle_system.hpp"

ParticleSystem::~ParticleSystem() {
    // The interactions were created in the heap
    // std::cout << "Destroying particle system " << class_allocated_interactions.size() << std::endl;
    for (int i = 0; i < class_allocated_interactions.size(); i++) {
        delete class_allocated_interactions[i];
    }
}

void ParticleSystem::resize_containers(unsigned int nDoF) {
    n_particles = nDoF / 3;
    x.resize(nDoF);
    v.resize(nDoF);
    v.setZero();
    x.setZero();
    fixed.resize(n_particles, false);
}

vec3 ParticleSystem::get_particle_position(int index) {
    return vec3(x(3*index), x(3*index + 1), x(3*index + 2));
}

vec3 ParticleSystem::get_particle_velocity(int index) {
    return vec3(v(3*index), v(3*index + 1), v(3*index + 2));
}

void ParticleSystem::fill_containers() {
    /* Fills the force, velocity and position vectors and mass and
     * force derivative matrices */

    // construct the mass matrix
    typedef Eigen::Triplet<double> tri;
    for (int i = 0; i < n_particles; i++) {
        // Mass identity matrix
        integrator->add_mass_triplet(tri(index + 3*i, index + 3*i, mass[i]));
        integrator->add_mass_triplet(tri(index + 3*i+1, index + 3*i+1, mass[i]));
        integrator->add_mass_triplet(tri(index + 3*i+2, index + 3*i+2, mass[i]));

        // Mass contribution to the equation matrix
        // equation_matrix = Mass_s - h * df_dv_s - h * h * df_dx_s;
        integrator->add_equation_triplet(tri(index + 3*i, index + 3*i, mass[i]));
        integrator->add_equation_triplet(tri(index + 3*i+1, index + 3*i+1, mass[i]));
        integrator->add_equation_triplet(tri(index + 3*i+2, index + 3*i+2, mass[i]));
    }

    // Fill the velocity and positions vectors
    for (int i = 0; i < nDoF; i++){
        integrator->x[index+i] = x(i);
        integrator->v[index+i] = v(i);
    }

    // Fill the force and derivative vectors
    for (int i = 0; i < interactions.size(); i++){
        interactions[i]->apply(*integrator, this);
    }
}

void ParticleSystem::update_state() {
    /* Update of the position & velocity vectors */
    for (int i=0; i < n_particles; i++) {
        for (int j=0; j <3; j++) {
            v[3*i+j] += integrator->delta_v[index + 3*i+j];
            x[3*i+j] += integrator->getTimeStep() * v[3*i+j];
        }
    }
}

void ParticleSystem::set_particle_position(int index, vec3& pos){
    x(3 * index) = pos.x();
    x(3 * index + 1) = pos.y();
    x(3 * index + 2) = pos.z();
}


void ParticleSystem::set_state() {
    for (int i=0; i < n_particles; i++) {
        for (int j=0; j <3; j++) {
            v[3*i+j] = integrator->v(index + 3*i+j);
            x[3*i+j] = integrator->x(index + 3*i+j);
        }
    }
}
