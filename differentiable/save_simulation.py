#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationRecorder
import numpy as np
import symulathon


def newton_iteration(x0, v0, xi, vi):
    symulathon.fill_containers()
    A = symulathon.get_equation_matrix()
    # b = symulathon.get_equation_vector()
    dfdx = symulathon.get_force_position_jacobian()
    f = symulathon.get_force()
    b = mass * v0 + h * f + h * dfdx @ (x0 - xi) - (mass - h**2 * dfdx) @ vi
    delta_v = solve_system(A, b)
    # M v0 + h fi + h df/dx (x0 – xi) - (M – h² df/dx) vi
    v1 = vi + delta_v
    x1 = x0 + h * v1

    symulathon.set_state(x1, v1)

    # Check newton iteration
    symulathon.fill_containers()
    f = symulathon.get_force()
    # print(f"2nd iteration check = {(mass @ (v1-v0) - h * f)[3:]}")
    return x1, v1


if __name__ == "__main__":
    rec = SimulationRecorder()
    print("Initializing")
    symulathon.initialize_scene(True)
    print("Initializing done")
    K_VALUE = 3
    RECORD_FRAMES = 500
    symulathon.restart_simulation([K_VALUE, K_VALUE / 100])
    h = symulathon.get_time_step()
    frames_count = 0
    while (not symulathon.window_should_close()) and (frames_count <= RECORD_FRAMES):
        frames_count += 1

        symulathon.fill_containers()
        x = symulathon.get_position()
        v = symulathon.get_velocity()
        mass = symulathon.get_mass_matrix()
        rec.record_timestep(x, v)

        A = symulathon.get_equation_matrix()
        b = symulathon.get_equation_vector()

        delta_v = solve_system(A, b)

        v1 = v + delta_v
        x1 = x + h * v1
        symulathon.set_state(x1, v1)
        # x2, v2 = newton_iteration(x, v, x1, v1)
        # symulathon.set_state(x2, v2)

        symulathon.process_input()
        symulathon.render_state()

    symulathon.terminate()
    print("Done")
