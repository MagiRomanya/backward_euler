#include "pysimulation.hpp"
#include "collision_ball.hpp"
#include "contact.hpp"
#include "mesh.h"
#include "object_manager.hpp"
#include "shader_path.h"
#include "spring_counter.hpp"
#include "vec3.hpp"
#include <cstddef>
#include <memory>
#include <vector>

PySimulation::PySimulation(double k, double k_bend, bool graphics) {
    graphical=graphics;

    integrator = std::make_unique<Integrator>(TimeStep);

    // Create simulable
    setUpCloth();
    mass_spring = std::make_unique<MassSpring>(integrator.get(), cloth, NODE_MASS, k, k_bend);
#ifdef ENABLE_CONTACT
    mass_spring->add_interaction(collision_ball->getContact());
    // mass_spring->add_interaction(collision_plane->getContact());
#endif

    // Fix corners
    const int index = M * (N - 1);
    mass_spring->fix_particle(0);
    mass_spring->fix_particle(index);
    integrator->fill_containers();
}

PySimulation::PySimulation(std::vector<double> k, std::vector<double> k_bend, bool graphics) {
    graphical=graphics;

    integrator = std::make_unique<Integrator>(TimeStep);

    // Create simulable
    setUpCloth();
    mass_spring = std::make_unique<MassSpring>(integrator.get(), cloth, NODE_MASS, k, k_bend);
#ifdef ENABLE_CONTACT
    mass_spring->add_interaction(collision_ball->getContact());
    // mass_spring->add_interaction(collision_plane->getContact());
#endif

    // Fix corners
    const int index = M * (N - 1);
    mass_spring->fix_particle(0);
    mass_spring->fix_particle(index);
    integrator->fill_containers();
}

PySimulation::PySimulation(double k, double k_bend, double tilt_angle, bool graphics) {
    graphical=graphics;

    integrator = std::make_unique<Integrator>(TimeStep);

    // Create simulable
    setUpCloth();
    mass_spring = std::make_unique<MassSpring>(integrator.get(), cloth, NODE_MASS, k, k_bend, tilt_angle);
#ifdef ENABLE_CONTACT
    mass_spring->add_interaction(collision_ball->getContact());
    // mass_spring->add_interaction(collision_plane->getContact());
#endif

    // Fix corners
    const int index = M * (N - 1);
    mass_spring->fix_particle(0);
    mass_spring->fix_particle(index);
    integrator->fill_containers();
}


PySimulation::~PySimulation() {
    // std::cout << "Diff paramters = "
    //           << integrator->diff_manager.get_size()
    //           << std::endl;
}

void PySimulation::setUpCloth() {
    const double step = 0.5;
    CreateGrid(mesh, N, M, step);
    omanager.loadMesh("cloth", mesh);

    if (graphical) {
        renderer = std::make_unique<Renderer>();

        renderer->change_orbital();

        omanager.loadShader("texture", SHADER_PATH "/test.v0.vert",
                           SHADER_PATH "/texture.frag");
        omanager.loadShader("color", SHADER_PATH "/test.v0.vert",
                           SHADER_PATH "/vert_color.frag");
        omanager.loadShader("normals", SHADER_PATH "/test.v0.vert",
                           SHADER_PATH "/normals.frag");
        omanager.loadShader("geo_normals", SHADER_PATH "/normals_geom.vert",
                           SHADER_PATH "/normals.geom", SHADER_PATH "/color.frag");
        omanager.loadTexture("gandalf", TEXTURE_PATH "/gandalf.png");
        omanager.loadTexture("floor", TEXTURE_PATH "/floor.jpg");

        cloth = omanager.createObject("cloth", "texture");
        cloth->useTexture("gandalf", omanager.getTextureID("gandalf"));
        renderer->addObject(cloth);

        SimpleMesh planeMesh;
        CreateGrid(planeMesh, 2, 2, 20);
        omanager.loadMesh("floor", planeMesh);
        Object* floor = omanager.createObject("floor", "texture");
        floor->useTexture("gandalf", omanager.getTextureID("floor"));
        floor->translation = glm::vec3(-25.0f, -1.05f, -8.0f*N);
        floor->scaling = glm::vec3(5);
        floor->updateModelMatrix();

    }
    else {
        cloth = omanager.createObject("cloth");
    }

    // Handle model matrix
    cloth->translation = glm::vec3(-0.5 * N, 1.5*M, -4.0f*N);
    cloth->scaling = glm::vec3(3.0);
    cloth->updateModelMatrix();

    // Plane contact definition
    vec3 normal = vec3(0, 1, 0);
    vec3 center = vec3(0, -2.0f, -4.0f*N);
#ifdef ENABLE_CONTACT
    float radius = 5.0f;
    if (graphical) {
        collision_ball = std::make_unique<CollisionBall>(center, radius, *renderer);
        // collision_plane = std::make_unique<CollisionPlane>(vec3(0.0f, -1.05f, 0.0f),
        //                                                    normalize(vec3(0.0f, 1.0f, 0.0f)),
        //                                                    *renderer);
    }
    else {
        collision_ball = std::make_unique<CollisionBall>(center, radius);
        // collision_plane = std::make_unique<CollisionPlane>(vec3(0.0f, -1.05f, 0.0f),
        //                                                    normalize(vec3(0.0f, 1.0f, 0.0f)));
    }
#endif
}

void PySimulation::render_state() {
    if (renderer) {
        renderer->cameraInput();
        renderer->render();
    }
}

std::vector<unsigned int> PySimulation::getSpringNodeIndices() {
    /* Return a list of pairwise indexs refering to the two nodes in the mesh which form siprings.
     *The length of the output will be 2 times the number of springs. */
    std::vector<unsigned int> indices;
    std::vector<Edge> internal;
    std::vector<Edge> external;
    mesh.boundary(internal, external);

    for (int i = 0; i < external.size(); i++) {
        indices.push_back(external[i].a);
        indices.push_back(external[i].b);
    }
    for (int i = 0; i < internal.size(); i+=2) {
        const Edge &e1 = internal[i];
        const Edge &e2 = internal[i + 1];
        indices.push_back(e1.a);
        indices.push_back(e1.b);
    }
    return indices;
}

std::vector<unsigned int> PySimulation::getBendSpringNodeIndices() {
    std::vector<unsigned int> indices;
    std::vector<Edge> internal;
    std::vector<Edge> external;
    mesh.boundary(internal, external);

    indices.reserve(internal.size());

    for (int i = 0; i < internal.size(); i+=2) {
        const Edge &e1 = internal[i];
        const Edge &e2 = internal[i + 1];
        indices.push_back(e1.opposite);
        indices.push_back(e2.opposite);
    }
    return indices;
}

