#include "raylib.h"
#include "system.h"
#include "spring.h"
#include "spring_list.h"
#include <cmath>
#include <iostream>
#include "clock.h"
#include "mesh.h"

// Dimensions of the particles grid
#define N 20
#define M 20

// NOTE: spring_list is being deprecated

// TODO: Añadir sistema de aristas para crear la cuadricula y para añadir muelles de flexión
// TODO: Muelles de flexión

// Define physical parameters of the simulation
#define K_SPRING 1000
#define NODE_MASS 100

void load_from_mesh(System &system, const SimpleMesh &mesh){
    /* Reads a mesh and treats the vertices as particles and the
     * edges as springs */

    const double k = K_SPRING;

    const unsigned int n_coord = 3 * mesh.vertices.size();
    system.update_dimensions(mesh.vertices.size());

    for (size_t i=0; i < n_coord; i+=3){
        glm::vec3 p = mesh.vertices[i/3].Position;
        system.x[i]   = p.x;
        system.x[i+1] = p.y;
        system.x[i+2] = p.z;
    }

    // Add the springs
    std::vector<Edge> internalEdges;
    std::vector<Edge> externalEdges;

    mesh.boundary(internalEdges, externalEdges);

    double L;
    for (size_t i=0; i < externalEdges.size(); i++){
        Edge &e = externalEdges[i];
        L = mesh.distance(e.a, e.b);
        system.interactions.push_back(new Spring(e.a, e.b, k, L));
    }

    for (size_t i=0; i < internalEdges.size(); i+=2){
        Edge &e1 = internalEdges[i];
        Edge &e2 = internalEdges[2];
        L = mesh.distance(e1.a, e1.b);
        system.interactions.push_back(new Spring(e1.a, e1.b, k, L));
    }

}

int main() {
    System system;

    // separation between the particles
    double step = 50;

    SimpleMesh mesh;
    CreateGrid(mesh, N, M, step);

    load_from_mesh(system, mesh);

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

    InitWindow(screenWidth, screenHeight, "Backward Euler");
    float radius = step / 10;

    SetTargetFPS(120);

    // RENDER LOOP
    while (!WindowShouldClose()) {

        system.update();

        if (IsKeyPressed(KEY_Q)) {
            CloseWindow();
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        system.render();

        DrawFPS(50, 50);
        DrawText(TextFormat("Energy: %d", system.energy), 50, screenHeight - 50, 20, BLACK);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
