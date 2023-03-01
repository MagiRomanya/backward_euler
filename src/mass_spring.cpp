#include "mass_spring.hpp"
#include "clock.h"

MassSpring::MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring)
{
    load_from_mesh(obj, node_mass, k_spring);
    integrator->add_simulable(this);
}

MassSpring::~MassSpring() {
    // The interactions were created in the heap
    for (int i = 0; i < interactions.size(); i++){
        delete interactions[i];
    }
}

void MassSpring::load_from_mesh(Object* obj, double node_mass, double k_spring) {
    /* Reads a mesh and treats the vertices as particles and the
     * edges as springs */
    this->mesh = obj->mesh;
    this->mesh->isDynamic = true;
    this->obj = obj;

    const double k = k_spring;
    const double k_flex = k/ 100;

    const unsigned int n_coord = 3 * mesh->vertices.size();
    n_particles = mesh->vertices.size();
    nDoF = 3 * n_particles;
    resize_containers(nDoF);

    // Mass for each node
    mass.resize(n_particles, node_mass);

    for (size_t i=0; i < n_coord; i+=3){
        glm::vec3 p = mesh->vertices[i/3].Position;
        x[i]   = p.x;
        x[i+1] = p.y;
        x[i+2] = p.z;
    }

    // Add the springs
    std::vector<Edge> internalEdges;
    std::vector<Edge> externalEdges;

    mesh->boundary(internalEdges, externalEdges);

    double L;
    for (size_t i=0; i < externalEdges.size(); i++){
        Edge &e = externalEdges[i];
        L = mesh->distance(e.a, e.b);
        interactions.push_back(new Spring(e.a, e.b, k, L));
    }

    for (size_t i=0; i < internalEdges.size(); i+=2){
        Edge &e1 = internalEdges[i];
        Edge &e2 = internalEdges[i+1];
        L = mesh->distance(e1.a, e1.b);
        // Normal spring
        interactions.push_back(new Spring(e1.a, e1.b, k, L));

        // Flex spring
        L = mesh->distance(e1.opposite, e2.opposite);
        interactions.push_back(new Spring(e1.opposite, e2.opposite, k_flex, L));
    }
}

void MassSpring::fill_containers() {
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

    // Change to the position coordinate system
    positions_to_world();

    // Fill the velocity and positions vectors
    for (int i = 0; i < nDoF; i++){
        integrator->x[index+i] = x(i);
        integrator->v[index+i] = v(i);
    }

    // Add gravity
    double gravity = mass[0]; // assuming all equal masses
    for (int i = 0; i < n_particles ; i += 1) {
        if (!fixed[i])
            integrator->f0[3*i + 1] += gravity; // z direction
    }
    // Fill the force and derivative vectors
    for (int i = 0; i < interactions.size(); i++){
        interactions[i]->apply(*integrator, this);
    }
}

void MassSpring::update_state() {
    /* Update of the position & velocity vectors */
    for (int i=0; i < n_particles; i++) {
        for (int j=0; j <3; j++) {
            v[3*i+j] += integrator->delta_v(index + 3*i+j);
            x[3*i+j] += integrator->getTimeStep() * v(3*i+j);
        }
    }
    positions_to_local();
    update_mesh();
}

void MassSpring::update_mesh() {
    /* Updates the positions of the mesh with the simulated data */
    if (n_particles - mesh->vertices.size() != 0) {
        std::cout << "ERROR::MASS_SPRING::UPDDATE_MESH: The number of particles and the number of vertices of the mesh are different" << std::endl;
    }
    // #pragma omp parallel for
    for (unsigned int i = 0; i < n_particles; i++){
        const vec3& current_pos = get_particle_position(i);
        glm::vec3 glm_pos = glm::vec3(current_pos.x(), current_pos.y(), current_pos.z());
        // glm_pos = obj->toLocal(glm_pos);
        mesh->vertices[i].Position = glm_pos;
    }
    mesh->updateVAO();
}

void MassSpring::positions_to_local() {
    // Convert to world coordinates
    for (int i = 0; i < n_particles; i++){
        glm::vec3 local_pos_glm = obj->toLocal(get_particle_position(i).to_glm());
        vec3 local_pos = to_vec3(local_pos_glm);
        set_particle_position(i, local_pos);
    }
}

void MassSpring::positions_to_world(){
    // Convert to world coordinates
    for (int i = 0; i < n_particles; i++){
        glm::vec3 world_pos_glm = obj->toWorld(get_particle_position(i).to_glm());
        vec3 world_pos = to_vec3(world_pos_glm);
        set_particle_position(i, world_pos);
    }
}
