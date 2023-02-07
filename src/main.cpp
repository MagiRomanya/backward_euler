#include "system.h"
#include "spring.h"
#include <cmath>
#include <iostream>
#include "clock.h"

#include "renderer.h"

// DONE: Añadir sistema de aristas para crear la cuadricula y para añadir muelles de flexión
// DONE: Muelles de flexión

// TODO Calcular las normales de la mesh en cada frame
//  - a nivel de facetas
//  - interpolar para tener normales a nivel de vertices
// TODO Introducir muelles de flexión basados en los angulos
// TODO Añadir luz phong al renderer

// Dimensions of the particles grid
#define N 20
#define M 20


// Define physical parameters of the simulation
#define K_SPRING 10000
#define NODE_MASS 100

Renderer renderer = Renderer();

int main() {
    System system;
    // separation between the particles
    double step = 50;

    SimpleMesh mesh;
    // mesh.loadFromFile("../../renderer/img/bunny.obj");
    CreateGrid(mesh, N, M, step);
    system.load_from_mesh(mesh, K_SPRING);
    system.Mass_s *= NODE_MASS;

    Shader shader = Shader("../shaders/test.v0.vert", "../shaders/test.v0.frag");

    Object cloth = Object(&system.mesh, shader);

    // Set up model matrix for the object
    cloth.translation = glm::vec3(0.0f, 0.0f, -4.0f);
    cloth.scaling = glm::vec3(0.005f);
    // cloth.scaling = glm::vec3(100.0f);
    cloth.updateModelMatrix();

    // Load a texture for the object
    cloth.loadTexture("gandalf", "../../renderer/img/gandalf.png");

    // Add the object to the renderer
    renderer.addObject(&cloth);

    // Fix corners
    const int index = M * (N - 1);
    system.fix_particle(0);
    system.fix_particle(index);
    system.h = 1;


    GLFWwindow* window = renderer.window;
    // glfwSetCursorPosCallback(window, mouse_callback);

    // RENDER LOOP
    while (!renderer.windowShouldClose()){
        // Input
        renderer.cameraInput();

        // Simulation step
        system.update();

        // Render
        renderer.render();

        // Swap buffers + poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    return 0;
}
