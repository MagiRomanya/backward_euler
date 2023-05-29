#include "mass_spring.hpp"
#include "clock.h"
#include "gravity.hpp"
#include "interaction.h"
#include "simulable.hpp"
#include "spring_test.hpp"

MassSpring::MassSpring(Integrator* integrator, Object* obj, double node_mass, double k_spring)
{
    k_flex = k_spring;
    k_bend = k_flex/ 100;
    nParameters = 2;
    load_from_mesh(obj, node_mass);
    gravity_vec = vec3(mass[0] * vec3(0, -1, 0));
    integrator->add_simulable(this);
}

MassSpring::MassSpring(Integrator* integrator, double node_mass, double k_spring)
{
    k_flex = k_spring;
    k_bend = k_flex/ 100;
    nParameters = 2;

    // Set up two particles with a spring
    n_particles = 2;
    nDoF = 3 * n_particles;
    mass.resize(n_particles, node_mass);
    resize_containers(nDoF);
    gravity_vec = vec3(node_mass * vec3(0, -1, 0));

    x[0] = 0; x[1] = 0; x[2] = 0;
    x[3] = 1; x[4] = 0; x[5] = 0;
    add_spring(0, 1, FLEX, 0);

    // Add gravity
    Interaction* gravity = new Gravity(&gravity_vec);
    add_interaction(gravity);
    class_allocated_interactions.push_back(gravity);

    integrator->add_simulable(this);
}

void MassSpring::load_from_mesh(Object* obj, double node_mass) {
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
    const unsigned int springs_index = interactions.size();
    const unsigned int n_springs = externalEdges.size() + 2 * internalEdges.size();

    for (size_t i=0; i < externalEdges.size(); i++){
        Edge &e = externalEdges[i];
        L = mesh->distance(e.a, e.b);
        add_spring(e.a, e.b, FLEX, L);
    }

    for (size_t i=0; i < internalEdges.size(); i+=2){
        Edge &e1 = internalEdges[i];
        Edge &e2 = internalEdges[i+1];
        L = mesh->distance(e1.a, e1.b);
        // Normal spring
        add_spring(e1.a, e1.b, FLEX, L);

        // Bend spring
        L = mesh->distance(e1.opposite, e2.opposite);
        add_spring(e1.opposite, e2.opposite, BEND, L);
    }

    // Add gravity
    Interaction* gravity = new Gravity(&gravity_vec);
    add_interaction(gravity);
    class_allocated_interactions.push_back(gravity);
}

void MassSpring::add_spring(unsigned int i1, unsigned int i2, SPRING_TYPE type, double L) {
    // Interaction* spring = new Spring(i1, i2, K, L);
    lengths.push_back(L);
    Interaction* spring;
    if  (type == FLEX) {
        double param[2] = {k_flex, lengths.back()};
        spring = new TestingSpring(i1, i2, param);
    }
    else {
        double param[2] = {k_bend, lengths.back()};
        spring = new TestingSpring(i1, i2, param);
    }

    add_interaction(spring);
    class_allocated_interactions.push_back(spring);
}

void MassSpring::fill_containers() {
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
    // Change to the world coordinate system
    // (only when we deal with meshes)
    if (current_coordinate_system == LOCAL && mesh != nullptr){
        positions_to_world();
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

void MassSpring::update_state() {
    /* Update of the position & velocity vectors */
    for (int i=0; i < n_particles; i++) {
        for (int j=0; j <3; j++) {
            v[3*i+j] += integrator->delta_v(index + 3*i+j);
            x[3*i+j] += integrator->getTimeStep() * v(3*i+j);
        }
    }
    update_mesh();
}

void MassSpring::update_mesh() {
    /* Updates the positions of the mesh with the simulated data */
    if (mesh == nullptr) return;
    if (current_coordinate_system == WORLD) {
        positions_to_local();
    }

    if (n_particles - mesh->vertices.size() != 0) {
        std::cout << "ERROR::MASS_SPRING::UPDDATE_MESH: The number of particles and the number of vertices of the mesh are different" << std::endl;
    }
    // #pragma omp parallel for
    for (unsigned int i = 0; i < n_particles; i++){
        const vec3& current_pos = get_particle_position(i);
        glm::vec3 glm_pos = glm::vec3(current_pos.x(), current_pos.y(), current_pos.z());
        mesh->vertices[i].Position = glm_pos;
    }
    mesh->calculate_vertex_normals();
    mesh->updateVAO();
}

void MassSpring::positions_to_local() {
    // Convert to mesh local coordinates
    current_coordinate_system = LOCAL;
    for (int i = 0; i < n_particles; i++){
        glm::vec3 local_pos_glm = obj->toLocal(get_particle_position(i).to_glm());
        vec3 local_pos = to_vec3(local_pos_glm);
        set_particle_position(i, local_pos);
    }
}

void MassSpring::positions_to_world(){
    // Convert to world coordinates
    current_coordinate_system = WORLD;
    for (int i = 0; i < n_particles; i++){
        glm::vec3 world_pos_glm = obj->toWorld(get_particle_position(i).to_glm());
        vec3 world_pos = to_vec3(world_pos_glm);
        set_particle_position(i, world_pos);
    }
}
