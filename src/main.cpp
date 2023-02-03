#include "system.h"
#include "spring.h"
#include "spring_list.h"
#include <cmath>
#include <iostream>
#include "clock.h"

#include "renderer.h"

// Dimensions of the particles grid
#define N 20
#define M 20

// NOTE: spring_list is being deprecated

// TODO: Añadir sistema de aristas para crear la cuadricula y para añadir muelles de flexión
// TODO: Muelles de flexión

// Define physical parameters of the simulation
#define K_SPRING 10000
#define NODE_MASS 100

void processInput(GLFWwindow* window);

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

Renderer renderer = Renderer();

int main() {
    System system;
    // separation between the particles
    double step = 50;

    SimpleMesh mesh;
    CreateGrid(mesh, N, M, step);

    Shader shader = Shader("/home/magi/Documents/Project/backward_euler/shaders/test.v0.vert",
                           "/home/magi/Documents/Project/backward_euler/shaders/test.v0.frag");

    system.load_from_mesh(mesh, K_SPRING, shader);

    system.object.translation = glm::vec3(0.0f, 0.0f, -4.0f);
    system.object.scaling = glm::vec3(0.005f);
    system.object.updateModelMatrix();

    system.object.view = renderer.camera.GetViewMatrix();
    system.object.proj = glm::perspective(30.0f, 1.0f, 0.1f, 100.f);
    system.object.loadTexture("gandalf", "../../renderer/img/gandalf.png");

    renderer.objects.push_back(&system.object);

    // Fix corners
    int index = M * (N - 1);
    system.fix_particle(0);
    system.fix_particle(index);
    system.h = 1;

    // Dense and sparse mass
    system.Mass *= NODE_MASS;
    system.Mass_s *= NODE_MASS;

    // RAYLIB RENDERING
    const int screenWidth = 1200;
    const int screenHeight = 1000;

    GLFWwindow* window = renderer.window;
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);

    float radius = step / 10;

    // RENDER LOOP
    while (!glfwWindowShouldClose(window)){
        // Input
        processInput(window);

        // Simulation step
        system.update();

        // Render
        renderer.render();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    return 0;
}
void processInput(GLFWwindow* window){
    /* Controlls user input through the window */
    if(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
        glfwSetWindowShouldClose(window, true);
    }

    float deltaTime = 0.0f;
    static float lastFrame = 0.0f;
    float currentFrame = glfwGetTime();
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        renderer.camera.ProcessKeyboard(FORWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        renderer.camera.ProcessKeyboard(BACKWARD, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        renderer.camera.ProcessKeyboard(LEFT, deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){
        renderer.camera.ProcessKeyboard(RIGHT, deltaTime);
    }

    // Enable / disable orbital cam
    static bool escape_last_frame = false;
    bool escape_this_frame = glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS;
    if (escape_this_frame and (escape_this_frame != escape_last_frame)){
        renderer.camera.is_orbital = !renderer.camera.is_orbital;
        if (renderer.camera.is_orbital)
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        else
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }
    escape_last_frame = escape_this_frame;
    glm::mat4 view = renderer.camera.GetViewMatrix();
    for (int i=0; i < renderer.objects.size(); i++){
        renderer.objects[i]->view = view;
    }
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height){
    /* Gets called every time we resize the window */
    glViewport(0, 0, width, height);
    for (int i = 0; i < renderer.objects.size(); i++)
        renderer.objects[i]->proj = glm::perspective(30.0f, ( (float) width) / height, 0.1f, 100.f);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos){
    static float lastX = 0.0f;
    static float lastY = 0.0f;
    float deltaX = xpos - lastX;
    float deltaY = ypos - lastY;
    const float threshold = 70.0f;
    deltaX = abs(deltaX) > threshold ? 0.0f: deltaX;
    deltaY = abs(deltaY) > threshold ? 0.0f: deltaY;
    lastX = xpos;
    lastY = ypos;
    renderer.camera.ProcessMouseMovement(deltaX, deltaY);
}
