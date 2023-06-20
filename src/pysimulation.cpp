#include "pysimulation.hpp"
#include "object_manager.hpp"
#include <memory>
#include <vector>

PySimulation::PySimulation(double k, double k_bend, bool graphics) {
    graphical=graphics;

    integrator = std::make_unique<Integrator>(TimeStep);

    // Create simulable
    setUpCloth();
    mass_spring = std::make_unique<MassSpring>(integrator.get(), &cloth, NODE_MASS, k, k_bend);

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
    mass_spring = std::make_unique<MassSpring>(integrator.get(), &cloth, NODE_MASS, k, k_bend);

    // Fix corners
    const int index = M * (N - 1);
    mass_spring->fix_particle(0);
    mass_spring->fix_particle(index);
    integrator->fill_containers();
}

void PySimulation::setUpCloth() {
    const double step = 0.5;
    CreateGrid(mesh, N, M, step);
    omanager.loadMesh("cloth", mesh);

    if (graphical) {
        renderer = std::make_unique<Renderer>();
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

        cloth = omanager.createObject("cloth", "normals", "geo_normals");
        cloth.useTexture("gandalf", omanager.getTextureID("gandalf"));
        renderer->addObject(&cloth);
    }
    else {
        cloth = omanager.createObject("cloth");
    }

    // Handle model matrix
    cloth.translation = glm::vec3(-0.5 * N, 0.0f, -4.0f*N);
    cloth.scaling = glm::vec3(3.0);
    cloth.updateModelMatrix();
}

void PySimulation::render_state() {
    if (renderer) {
        renderer->cameraInput();
        renderer->render();
    }
}
