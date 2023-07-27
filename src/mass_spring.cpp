#include "mass_spring.hpp"
#include "Eigen/src/SparseCore/SparseUtil.h"
#include "glm/trigonometric.hpp"
#include "gravity.hpp"
#include "integrator.hpp"
#include "mesh.h"
#include "parameter.hpp"
#include "parameter_list.hpp"
#include "spring_test.hpp"
#include <cmath>
#include <vector>

MassSpring::MassSpring(Integrator *integrator, Object *obj, double node_mass, double k_spring) {
    double bend_multiplyer = 1.0/100.0;
    // Parameters
    normalSpringParameters.addParameter(&integrator->diff_manager, k_spring); // k (diff)
    normalSpringParameters.addParameter(1.0f); // L
    normalSpringParameters.addParameter(1.0f); // alpha
    bendSpringParameters = normalSpringParameters;
    bendSpringParameters.updateParameter(2, bend_multiplyer); // alpha

    load_from_mesh(integrator, obj, node_mass);
    gravity_vec = mass[0] * vec3(0, -1, 0);
    integrator->add_simulable(this);
}

MassSpring::MassSpring(Integrator *integrator, Object *obj, double node_mass, double k_spring, double k_bend) {
    // Parameters
    /// Normal Spring Parameter
    normalSpringParameters.addParameter(&integrator->diff_manager, k_spring); // diff
    normalSpringParameters.addParameter(1.0f);
    normalSpringParameters.addParameter(1.0f);

    /// Bend Spring Parameter
    bendSpringParameters.addParameter(&integrator->diff_manager, k_bend); // diff
    bendSpringParameters.addParameter(1.0f);
    bendSpringParameters.addParameter(1.0f);

    load_from_mesh(integrator, obj, node_mass);
    gravity_vec = mass[0] * vec3(0, -1, 0);
    integrator->add_simulable(this);
}

MassSpring::MassSpring(Integrator* integrator, Object* obj, double node_mass,
                       const std::vector<double>& k_spring,
                       const std::vector<double>& k_bend) {
    vector_paramters = true;
    // Tension / flex springs
    for (int i = 0; i < k_spring.size(); i++) {
        ParameterList pl;
        pl.addParameter(&integrator->diff_manager, (k_spring[i])); // k
        pl.addParameter(1); // L
        pl.addParameter(1); // Alpha
        spring_parameters.push_back(pl);
    }
    // Bend springs
    for (int i = 0; i < k_bend.size(); i++) {
        ParameterList pl;
        pl.addParameter(&integrator->diff_manager, (k_bend[i])); // k
        pl.addParameter(1); // L
        pl.addParameter(1); // Alpha
        spring_parameters.push_back(pl);
    }

    load_from_mesh(integrator, obj, node_mass);
    gravity_vec = mass[0] * vec3(0, -1, 0);
    integrator->add_simulable(this);
}

MassSpring::MassSpring(Integrator* integrator, Object* obj, double node_mass,
                       double k_spring, double k_bend, double tilt_angle) {
    // Parameters
    /// Normal Spring Parameter
    normalSpringParameters.addParameter(&integrator->diff_manager, k_spring); // k_elasticity (diff)
    normalSpringParameters.addParameter(1.0f); // rest longitude
    normalSpringParameters.addParameter(1.0f); // alpha multiplicator

    /// Bend Spring Parameter
    bendSpringParameters.addParameter(&integrator->diff_manager, k_bend); // (diff)
    bendSpringParameters.addParameter(1.0f);
    bendSpringParameters.addParameter(1.0f);

    this->tilt_angle = Parameter(&integrator->diff_manager, tilt_angle); // (diff)
    load_from_mesh(integrator, obj, node_mass);
    gravity_vec = mass[0] * vec3(0, -1, 0);
    integrator->add_simulable(this);
}

void MassSpring::load_from_mesh(Integrator* itg, Object *obj, double node_mass) {
    /* Reads a mesh and treats the vertices as particles and the
     * edges as springs */
    SimpleMesh* mesh = obj->mesh;
    mesh->isDynamic = true;
    this->obj = obj;
    obj->rotation = glm::vec3(tilt_angle.getValue(), 0, 0);
    obj->updateModelMatrix();

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
    bend_offset = internalEdges.size() / 2.0 + externalEdges.size();
    double L;

    for (size_t i = 0; i < externalEdges.size(); i++) {
        Edge &e = externalEdges[i];
        L = mesh->distance(e.a, e.b, obj->model);
        add_spring(itg, e.a, e.b, FLEX, L);
    }

    for (size_t i = 0; i < internalEdges.size(); i += 2) {
        Edge &e1 = internalEdges[i];
        Edge &e2 = internalEdges[i + 1];
        L = mesh->distance(e1.a, e1.b, obj->model);
        // Normal spring
        add_spring(itg, e1.a, e1.b, FLEX, L);

        // Bend spring
        L = mesh->distance(e1.opposite, e2.opposite, obj->model);
        add_spring(itg, e1.opposite, e2.opposite, BEND, L);
    }

    // Add gravity
    Interaction *gravity = new Gravity(&gravity_vec);
    add_interaction(gravity);
    class_allocated_interactions.push_back(gravity);
}

void MassSpring::add_spring(Integrator* itg, unsigned int i1, unsigned int i2, SPRING_TYPE type, double L) {
    Interaction *spring;
    if (!vector_paramters) {
        if (type == FLEX) {
            normalSpringParameters.updateParameter(1, L);
            spring = new TestingSpring(i1, i2, normalSpringParameters);
        } else {
            bendSpringParameters.updateParameter(1, L);
            spring = new TestingSpring(i1, i2, bendSpringParameters);
        }
    }
    else {
        if (type == FLEX) {
            spring_parameters.at(flex_counter).updateParameter(1, L);
            spring = new TestingSpring(i1, i2, spring_parameters[flex_counter]);
            flex_counter++;
        } else {
            spring_parameters.at(bend_offset + bend_counter).updateParameter(1, L);
            spring = new TestingSpring(i1, i2, spring_parameters[bend_offset + bend_counter]);
            bend_counter++;
        }
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
    if (obj == nullptr)
        return;

    SimpleMesh* mesh = obj->mesh;

    // We need local frame to update the mesh model matrix
    positions_to_local();

    if (n_particles - mesh->vertices.size() != 0) {
        std::cerr << "ERROR::MASS_SPRING::UPDDATE_MESH: The number of particles "
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
    // update graphics
    mesh->updateVAO();

    // Now that the mesh has been updated we need to go back to world frame to do the simulation
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

void MassSpring::get_initial_state_jacobian(Eigen::SparseMatrix<double>& dx0dp, Eigen::SparseMatrix<double>& dv0dp) {
    // Initialize velocities to zero
    dv0dp.resize(nDoF, integrator->diff_manager.get_size());
    dx0dp.resize(nDoF, integrator->diff_manager.get_size());

    if (tilt_angle.isDiff() && n_particles > 0) {
        // Calculate the position derivatives wrt angle
        std::vector<Eigen::Triplet<double>> dx0dp_triplets;
        const vec3 origin = vec3(x[0], x[1], x[2]);

        for (size_t i = 0; i < n_particles; i++) {
            const vec3 point = vec3(x[3*i], x[3*i+1], x[3*i+2]);
            const vec3 delta = point - origin;
            // const double L = sqrt(delta.y()*delta.y() + delta.z()*delta.z());
            // std::cout << "L value = " << L << std::endl;
            // x coordinate not affected by x axis rotation

            // y coordinate
            dx0dp_triplets.push_back(Eigen::Triplet<double>(3*i+1, tilt_angle.getIndex(), delta.z()));

            // z coordinate
            dx0dp_triplets.push_back(Eigen::Triplet<double>(3*i+2, tilt_angle.getIndex(), -delta.y()));

            dx0dp.setFromSortedTriplets(dx0dp_triplets.begin(), dx0dp_triplets.end());
        }

    }
}
