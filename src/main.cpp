#include <cmath>
#include <iostream>

#include "mass_spring.hpp"
#include "integrator.hpp"
#include "clock.h"
#include "contact.h"

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
#define K_SPRING 100000
#define NODE_MASS 100


int main() {
    ///////////////// CREATE RENDERER & MESHES /////////////////////
    Renderer renderer;
    double step = 0.5;
    ObjectManager manager;
    SimpleMesh mesh;
    SimpleMesh planeMesh;

    CreateGrid(mesh, N, M, step);
    CreateGrid(planeMesh, N, M, step);

    manager.loadMesh("cloth", mesh);
    manager.loadMesh("plane", planeMesh);
    manager.loadShader("test",SHADER_PATH"/test.v0.vert", SHADER_PATH"/test.v0.frag");
    manager.loadTexture("gandalf", TEXTURE_PATH"/gandalf.png");

    Object floor = manager.createObject("plane", "test");
    Object cloth = manager.createObject("cloth", "test");
    cloth.useTexture("gandalf", manager.getTextureID("gandalf"));

    // Set up model matrix for the object
    cloth.translation = glm::vec3(0.0f, 0.5*N / 2.0f , -4.0f);
    cloth.updateModelMatrix();

    floor.translation = glm::vec3(500.0f, -1.0f, 500.0f);
    floor.rotation = glm::vec3(0.0f);
    floor.scaling = glm::vec3(100);
    floor.updateModelMatrix();

    // Add the object to the renderer
    renderer.addObject(&cloth);
    renderer.addObject(&floor);

    ////////////////// ADD INTEGRATOR & SIMULABLES ////////////////////
    Integrator integrator(0.1f);
    MassSpring mass_spring = MassSpring(&integrator , &cloth, NODE_MASS, K_SPRING);

    InfPlane plane;
    plane.normal = vec3(0, 1, 0);
    plane.center = vec3(0, 0, 0);
    Contact planeContact(plane);
    mass_spring.add_interaction(&planeContact);


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
