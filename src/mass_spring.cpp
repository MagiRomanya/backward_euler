#include "mass_spring.hpp"

MassSpring::MassSpring(Integrator* integrator, SimpleMesh* mesh, double node_mass, double k_spring)
{
    load_from_mesh(mesh, node_mass, k_spring);
    integrator->add_simulable(this);
}

MassSpring::~MassSpring() {
    // The interactions were created in the heap
    for (int i = 0; i < interactions.size(); i++){
        delete interactions[i];
    }
}

void MassSpring::load_from_mesh(SimpleMesh* mesh, double node_mass, double k_spring) {
    /* Reads a mesh and treats the vertices as particles and the
     * edges as springs */
    this->mesh = mesh;
    this->mesh->isDynamic = true;

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

void MassSpring::update_mesh() {
    /* Updates the positions of the mesh with the simulated data */
    if (n_particles - mesh->vertices.size() != 0) {
        std::cout << "ERROR::MASS_SPRING::UPDDATE_MESH: The number of particles and the number of vertices of the mesh are different" << std::endl;
    }
    for (unsigned int i = 0; i < n_particles; i++){
        const vec3& current_pos = get_particle_position(i);
        mesh->vertices[i].Position = glm::vec3(current_pos.x(), current_pos.y(), current_pos.z());
    }
    mesh->updateVAO();
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
