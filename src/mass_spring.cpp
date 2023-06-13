#include "mass_spring.hpp"
#include "clock.h"
#include "gravity.hpp"
#include "interaction.h"
#include "simulable.hpp"
#include "spring_test.hpp"

MassSpring::MassSpring(Integrator *integrator, Object *obj, double node_mass, double k_spring) {
    k_flex = k_spring;
    k_bend = compute_k_bend();
    nParameters = 2;
    load_from_mesh(obj, node_mass);
    gravity_vec = mass[0] * vec3(0, -1, 0);
    // gravity_vec = 0.0f * vec3(0, -1, 0);
    integrator->add_simulable(this);
}

MassSpring::MassSpring(Integrator *integrator, double node_mass, double k_spring) {
    k_flex = k_spring;
    k_bend = compute_k_bend();
    nParameters = 2;

    // Set up two particles with a spring
    n_particles = 4;
    nDoF = 3 * n_particles;
    mass.resize(n_particles, node_mass);
    resize_containers(nDoF);
    gravity_vec = vec3(node_mass * vec3(0, -1, 0));
    // gravity_vec = 0.0f * vec3(0, -1, 0);

    x[0] = 0;
    x[1] = 0.5;
    x[2] = -4;

    x[3] = 0;
    x[4] = 0.5;
    x[5] = -2.5;

    x[6] = 1.5;
    x[7] = 0.5;
    x[8] = -4;

    x[9] = 1.5;
    x[10] = 0.5;
    x[11] = -2.5;
    add_spring(0, 2, FLEX, (get_particle_position(0) - get_particle_position(2)).length());
    add_spring(2, 3, FLEX, (get_particle_position(2) - get_particle_position(3)).length());
    add_spring(3, 1, FLEX, (get_particle_position(3) - get_particle_position(1)).length());
    // add_spring(2, 1, BEND, (get_particle_position(2) - get_particle_position(1)).length());
    fix_particle(0);
    fix_particle(1);

    // Add gravity
    Interaction *gravity = new Gravity(&gravity_vec);
    add_interaction(gravity);
    class_allocated_interactions.push_back(gravity);

    integrator->add_simulable(this);
}

void MassSpring::load_from_mesh(Object *obj, double node_mass) {
    /* Reads a mesh and treats the vertices as particles and the
     * edges as springs */
    this->mesh = obj->mesh;
    this->mesh->isDynamic = true;
    this->obj = obj;

    const unsigned int n_coord = 3 * mesh->vertices.size();
    n_particles = mesh->vertices.size();
    nDoF = 3 * n_particles;
    resize_containers(nDoF);

    // Mass for each node
    mass.resize(n_particles, node_mass);

    // Node positions
    for (size_t i = 0; i < n_coord; i += 3) {
        glm::vec3 p = mesh->vertices[i / 3].Position; // local coordinates
        x[i] = p.x;
        x[i + 1] = p.y;
        x[i + 2] = p.z;
    }
    // The simulation must be done in the world coordinate system
    positions_to_world();

    // Add the springs
    std::vector<Edge> internalEdges;
    std::vector<Edge> externalEdges;

    mesh->boundary(internalEdges, externalEdges);
    double L;
    const unsigned int springs_index = interactions.size();
    const unsigned int n_springs = externalEdges.size() + 2 * internalEdges.size();

    for (size_t i = 0; i < externalEdges.size(); i++) {
        Edge &e = externalEdges[i];
        L = mesh->distance(e.a, e.b);
        add_spring(e.a, e.b, FLEX, L);
    }

    for (size_t i = 0; i < internalEdges.size(); i += 2) {
        Edge &e1 = internalEdges[i];
        Edge &e2 = internalEdges[i + 1];
        L = mesh->distance(e1.a, e1.b);
        // Normal spring
        add_spring(e1.a, e1.b, FLEX, L);

        // Bend spring
        L = mesh->distance(e1.opposite, e2.opposite);
        add_spring(e1.opposite, e2.opposite, BEND, L);
    }

    // Add gravity
    Interaction *gravity = new Gravity(&gravity_vec);
    add_interaction(gravity);
    class_allocated_interactions.push_back(gravity);

}

void MassSpring::add_spring(unsigned int i1, unsigned int i2, SPRING_TYPE type,
                            double L) {
    // Interaction* spring = new Spring(i1, i2, K, L);
    lengths.push_back(L);
    Interaction *spring;
    if (type == FLEX) {
        double param[2] = {k_flex, lengths.back()};
        spring = new TestingSpring(i1, i2, param);
    } else {
        double param[2] = {k_bend, lengths.back()};
        spring = new TestingSpring(i1, i2, param);
    }

    add_interaction(spring);
    class_allocated_interactions.push_back(spring);
}

void MassSpring::update_state() {
    ParticleSystem::update_state();
    update_mesh();
}

void MassSpring::set_state() {
    ParticleSystem::set_state();
    update_mesh();
}

void MassSpring::update_mesh() {
    /* Updates the positions of the mesh with the simulated data */
    if (mesh == nullptr)
        return;

    // We need local frame to update the mesh model matrix
    positions_to_local();

    if (n_particles - mesh->vertices.size() != 0) {
        std::cout << "ERROR::MASS_SPRING::UPDDATE_MESH: The number of particles "
            "and the number of vertices of the mesh are different"
                  << std::endl;
    }
    // #pragma omp parallel for
    for (unsigned int i = 0; i < n_particles; i++) {
        const vec3 &current_pos = get_particle_position(i);
        glm::vec3 glm_pos =
            glm::vec3(current_pos.x(), current_pos.y(), current_pos.z());
        mesh->vertices[i].Position = glm_pos;
    }
    mesh->calculate_vertex_normals();
    mesh->updateVAO();

    // Now that the mesh has been updated we need to go back to world frame to properlly simulate
    positions_to_world();
}

void MassSpring::positions_to_local() {
    // Convert to mesh local coordinates
    for (int i = 0; i < n_particles; i++) {
        glm::vec3 local_pos_glm = obj->toLocal(get_particle_position(i).to_glm());
        vec3 local_pos = to_vec3(local_pos_glm);
        set_particle_position(i, local_pos);
    }
}

void MassSpring::positions_to_world() {
    // Convert to world coordinates
    for (int i = 0; i < n_particles; i++) {
        glm::vec3 world_pos_glm = obj->toWorld(get_particle_position(i).to_glm());
        vec3 world_pos = to_vec3(world_pos_glm);
        set_particle_position(i, world_pos);
    }
}
