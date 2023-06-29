#include <cmath>
#include <iostream>

#include "mass_spring.hpp"
#include "integrator.hpp"
#include "clock.h"
#include "contact.h"

#include "mass_spring_gui.hpp"
#include "renderer.h"
#include "object_manager.hpp"
#include "rigid_body.hpp"
#include "utilities.h"

// DONE: Añadir sistema de aristas para crear la cuadricula y para añadir muelles de flexión
// DONE: Muelles de flexión
// DONE Introducir contacto:
//  - Punto <-> plano
//  - Punto <-> esfera
// DONE Añadir luz phong al renderer
// DONE Hacer que el simulador sea cómo un servidor -> pybind
// TODO Introducir muelles de flexión basados en los angulos

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

    // manager.loadMesh("bunny", "/home/magi/Escriptori/dani.obj");
    manager.loadMesh("bunny", MESH_PATH"/bunny.obj");
    manager.loadMesh("cloth", mesh);
    manager.loadMesh("plane", planeMesh);

    manager.loadShader("texture",SHADER_PATH"/test.v0.vert", SHADER_PATH"/texture.frag");
    manager.loadShader("color",SHADER_PATH"/test.v0.vert", SHADER_PATH"/vert_color.frag");
    manager.loadShader("normals",SHADER_PATH"/test.v0.vert", SHADER_PATH"/normals.frag");
    manager.loadShader("geo_normals", SHADER_PATH"/normals_geom.vert", SHADER_PATH"/normals.geom", SHADER_PATH"/color.frag");

    manager.loadTexture("gandalf", TEXTURE_PATH"/fabric/FabricPlainGreyFlat015_COL_2K.jpg");
    manager.loadTexture("floor", TEXTURE_PATH"/cobblestone/GroundCobblestone001_COL_2K.jpg");

    Object* floor = manager.createObject("plane", "texture");
    Object* cloth = manager.createObject("cloth", "texture", "geo_normals");
    Object* bunny = manager.createObject("bunny", "normals");


    cloth->useTexture("gandalf", manager.getTextureID("gandalf"));
    floor->useTexture("gandalf", manager.getTextureID("floor"));

    // Set up model matrix for the object
    cloth->translation = glm::vec3(0.0f, 0.25 * N, -4.0f);
    // cloth.scaling = glm::vec3(3.0);
    cloth->updateModelMatrix();

    floor->translation = glm::vec3(-25.0f, -1.05f, -25.0f);
    floor->scaling = glm::vec3(5);
    floor->updateModelMatrix();

    // Add the object to the renderer
    renderer.addObject(cloth);
    renderer.addObject(floor);
    renderer.addObject(bunny);

    ////////////////// ADD INTEGRATOR & SIMULABLES ////////////////////
    Integrator integrator(0.05f);
    MassSpring mass_spring = MassSpring(&integrator , cloth, NODE_MASS, K_SPRING);
    MassSpringGUI mass_spring_gui = MassSpringGUI(&mass_spring, &renderer);

    InfPlane plane;
    plane.normal = vec3(0, 1, 0);
    plane.center = vec3(0, -1, 0);
    Contact planeContact(plane);
    mass_spring.add_interaction(&planeContact);

    RigidBody rb = RigidBody(&integrator, bunny, 100.0f);

    // Fix corners
    const int index = M * (N - 1);
    mass_spring.fix_particle(0);
    mass_spring.fix_particle(index);

    bool paused = true;
    SimpleTrigger<bool> t1(false);

    // RENDER LOOP
    while (!renderer.windowShouldClose()){
        // Input
        renderer.cameraInput();
        if (t1.changed_to_true(glfwGetKey(renderer.window, GLFW_KEY_SPACE) == GLFW_PRESS)){
            if (paused)
                std::cout << "Simulation paused." << std::endl;
            else
                std::cout << "Simulation continue." << std::endl;
            paused = ! paused;
        }

        // Simulation step
        if (!paused)
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
