#include "raylib.h"
#include "spring.h"
#include "spring_list.h"
#include "system.h"
#include <cmath>
#include <iostream>
#include "clock.h"

// Dimensions of the particles grid
#define N 20
#define M 20

void initialize_grid(System &s, Spring_list &springs, double step) {
  // Gives value to the starting positions in a greed manner
  // the greed is in te y-z plane
  // And also joins the particles with springs

  // Starting coordinates and separation between nodes
  double z_in, y_in;
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
      int index = M * i + j; // Particle index
      // Position components
      s.x(3 * index) = 0.0;                 // x
      s.x(3 * index + 1) = y_in + step * i; // y
      s.x(3 * index + 2) = z_in + step * j; // z

      // Velocity components
      s.v(3 * index) = 0.0;     // vx
      s.v(3 * index + 1) = 0.0; // vy
      s.v(3 * index + 2) = 0.0; // vz
    }
  }

  double k = 100;
  double L0 = step * 0.8;
  // Now it is time to join with springs
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < M; j++) {
      int index = M * i + j; // Particle index
      if (i != N - 1) {
        int i1 = M * (i + 1) + j;
        springs.add_spring(index, i1, k, L0);
      }
      if (j != M - 1) {
        int i2 = M * i + j + 1;
        springs.add_spring(index, i2, k, L0);
      }
    }
  }
}

void integration_step(System &s, Spring_list &springs) {
  // Set all forces and derivatives to zero
  s.f0.setZero();
  s.df_dx.setZero();
  s.df_dv.setZero();
  s.df_dv_s.setZero();
  s.df_dx_s.setZero();
  // Calculate forces and derivatives
  springs.add_spring_forces(s);
  springs.add_spring_derivatives(s);
  // springs.spring_derivatives_finite(s);

  // Add gravity
  double gravity = s.Mass(0, 0); // assuming all equal masses
  for (int i = 0; i < N * M * 3; i += 3) {
    s.f0[i + 2] += gravity; // z direction
  }

  // Fix extremes
  int index = M * (N - 1);
  s.fixed[0] = true;
  s.fixed[index] = true;

  // Compute a solution using backward euler
  // s.backward_euler();
  s.backward_euler_sparse();
  // Update velocity and positons
  s.update_vel_and_pos();
}

int main() {
  System system(M * N);
  Spring_list springs;

  system.h = 1.;
  system.Mass *= 100;

  // separation between the particles
  double step = 50;

  initialize_grid(system, springs, step);

  // RAYLIB RENDERING
  const int screenWidth = 1200;
  const int screenHeight = 1000;

  InitWindow(screenWidth, screenHeight, "Backward Euler");
  float radius = step / 10;

  SetTargetFPS(120);

  // RENDER LOOP
  while (!WindowShouldClose()) {

    integration_step(system, springs);

    if (IsKeyPressed(KEY_Q)) {
      CloseWindow();
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);
    // Drawing nodes
    for (int i = 0; i < N * M * 3; i += 3) {
      DrawCircle(system.x[i + 1] + 100, system.x[i + 2] + 100, radius,
                 (Color){0, 0, 0, 255});
      DrawCircle(system.x[i + 1] + 100, system.x[i] + 100, radius,
                 (Color){0, 0, 0, 255}); // 3d pesrspective
    }
    // Drawing connections between nodes
    for (int i = 0; i < springs.springs.size(); i++) {
      double sPosX = 100 + system.x[3 * springs.springs[i].i + 1];
      double sPosY = 100 + system.x[3 * springs.springs[i].i + 2];
      double ePosX = 100 + system.x[3 * springs.springs[i].j + 1];
      double ePosY = 100 + system.x[3 * springs.springs[i].j + 2];
      DrawLine(sPosX, sPosY, ePosX, ePosY, BLACK);
    }
    DrawFPS(50, 50);
    EndDrawing();
  }

  Clock last("last");
  last.printClocks();
  return 0;
}
