#include "mass_spring.hpp"
#include "integrator.hpp"
#include <cmath>
#include <iostream>
#include "clock.h"

#include "renderer.h"
#include "object_manager.hpp"

// DONE: Añadir sistema de aristas para crear la cuadricula y para añadir muelles de flexión
// DONE: Muelles de flexión

// TODO Calcular las normales de la mesh en cada frame
//  - a nivel de facetas
//  - interpolar para tener normales a nivel de vertices
// TODO Introducir muelles de flexión basados en los angulos
// TODO Añadir luz phong al renderer
// TODO Introducir contacto:
//  - Punto <-> plano
//  - Punto <-> esfera

// Dimensions of the particles grid
#define N 20
#define M 20


// Define physical parameters of the simulation
#define K_SPRING 10000
#define NODE_MASS 100

Renderer renderer = Renderer();

int main() {
    Integrator integrator(1.0f);
    // separation between the particles
    double step = 50;

    ObjectManager manager;
    SimpleMesh mesh;
    // mesh.loadFromFile("../../renderer/img/bunny.obj");
    CreateGrid(mesh, N, M, step);

    manager.loadMesh("cloth", mesh);
    MassSpring mass_spring = MassSpring(&integrator ,manager.getMesh("cloth"), NODE_MASS, K_SPRING);

    manager.loadShader("test",SHADER_PATH"/test.v0.vert", SHADER_PATH"/test.v0.frag");
    manager.loadTexture("gandalf", TEXTURE_PATH"/gandalf.png");

    Object cloth = manager.createObject("cloth", "test");
    cloth.useTexture("gandalf", manager.getTextureID("gandalf"));

    // Set up model matrix for the object
    cloth.translation = glm::vec3(0.0f, 0.0f, -4.0f);
    cloth.scaling = glm::vec3(0.005f);
    cloth.updateModelMatrix();

    // Add the object to the renderer
    renderer.addObject(&cloth);

    // Fix corners
    const int index = M * (N - 1);
    mass_spring.fix_particle(0);
    mass_spring.fix_particle(index);

    // RENDER LOOP
    while (!renderer.windowShouldClose()){
        // Input
        renderer.cameraInput();

        // Simulation step
        {
            Clock timer("Simulation Step");
            integrator.integration_step();
        }

        // Render
        renderer.render();
    }
    Clock results("result");
    results.printClocks();
    return 0;
}
