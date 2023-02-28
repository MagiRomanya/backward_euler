#include "mass_spring.hpp";

MassSpring::MassSpring(SimpleMesh* mesh, double k_spring) {
    load_from_mesh(mesh, k_spring);
}

MassSpring::~MassSpring() {
    // The interactions were created in the heap
    for (int i = 0; i < interactions.size(); i++){
        delete interactions[i];
    }
}

void MassSpring::load_from_mesh(SimpleMesh* mesh, double k_spring) {
    /* Reads a mesh and treats the vertices as particles and the
     * edges as springs */
    this->mesh = mesh;
    this->mesh->isDynamic = true;

    const double k = k_spring;
    const double k_flex = k/ 100;

    const unsigned int n_coord = 3 * mesh->vertices.size();
    resize_containers(mesh->vertices.size());

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
