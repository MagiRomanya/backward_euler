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
// DONE Introducir contacto:
//  - Punto <-> plano
//  - Punto <-> esfera
// TODO Hacer que el simulador sea cómo un servidor -> pybind
// TODO Introducir muelles de flexión basados en los angulos
// TODO Añadir luz phong al renderer

// Dimensions of the particles grid
#define N 20
#define M 20


// Define physical parameters of the simulation
#define K_SPRING 100000
#define NODE_MASS 100


int main() {
    ///////////////// CREATE RENDERER & MESHES /////////////////////
    Renderer renderer;
    ObjectManager manager;

    double step = 0.5;
    SimpleMesh mesh;
    SimpleMesh planeMesh;
    CreateGrid(mesh, N, M, step);
    CreateGrid(planeMesh, N, M, step);

    manager.loadMesh("bunny", TEXTURE_PATH"/bunny.obj");
    manager.loadMesh("cloth", mesh);
    manager.loadMesh("plane", planeMesh);

    manager.loadShader("texture",SHADER_PATH"/test.v0.vert", SHADER_PATH"/texture.frag");
    manager.loadShader("color",SHADER_PATH"/test.v0.vert", SHADER_PATH"/color.frag");
    manager.loadShader("normals",SHADER_PATH"/test.v0.vert", SHADER_PATH"/normals.frag");
    manager.loadShader("geo_normals", SHADER_PATH"/normals_geom.vert", SHADER_PATH"/normals.geom", SHADER_PATH"/color.frag");

    manager.loadTexture("gandalf", TEXTURE_PATH"/gandalf.png");
    manager.loadTexture("suelo", TEXTURE_PATH"/suelo.jpg");

    Object floor = manager.createObject("plane", "texture");
    Object cloth = manager.createObject("cloth", "texture", "geo_normals");

    cloth.useTexture("gandalf", manager.getTextureID("gandalf"));
    floor.useTexture("gandalf", manager.getTextureID("suelo"));

    // Set up model matrix for the object
    cloth.translation = glm::vec3(0.0f, 0.5*N / 2.0f , -4.0f);
    // cloth.scaling = glm::vec3(3.0);
    cloth.updateModelMatrix();

    floor.translation = glm::vec3(25.0f, -1.05f, 25.0f);
    floor.rotation = glm::vec3(0.0f);
    floor.scaling = glm::vec3(5);
    floor.updateModelMatrix();

    // Add the object to the renderer
    renderer.addObject(&cloth);
    renderer.addObject(&floor);

    ////////////////// ADD INTEGRATOR & SIMULABLES ////////////////////
    Integrator integrator(0.05f);
    MassSpring mass_spring = MassSpring(&integrator , &cloth, NODE_MASS, K_SPRING);

    InfPlane plane;
    plane.normal = vec3(0, -1, 0);
    plane.center = vec3(0, -1, 0);
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
