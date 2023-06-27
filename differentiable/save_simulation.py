#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationRecorder
import numpy as np
from interpolate_values import generate_parameters
from symulathon import Simulation


def newton_iteration(sim: Simulation, x0, v0, xi, vi):
    sim.fill_containers()
    A = sim.getEquationMatrix()
    # b = symulathon.get_equation_vector()
    dfdx = sim.getForcePositionJacobian()
    f = sim.getForce()
    b = mass * v0 + h * f + h * dfdx @ (x0 - xi) - (mass - h**2 * dfdx) @ vi
    delta_v = solve_system(A, b)
    # M v0 + h fi + h df/dx (x0 – xi) - (M – h² df/dx) vi
    v1 = vi + delta_v
    x1 = x0 + h * v1

    sim.set_state(x1, v1)

    # Check newton iteration
    sim.fill_containers()
    f = sim.getForce()
    # print(f"2nd iteration check = {(mass @ (v1-v0) - h * f)[3:]}")
    return x1, v1


if __name__ == "__main__":
    rec = SimulationRecorder("test")
    K_VALUE = 3
    RECORD_FRAMES = 500
    k, k_bend = generate_parameters((0.1, 1, 10, 0.1),
                                    (0.1, 0.1, 0.1, 0.1))
    sim = Simulation(k, k_bend, True)
    h = sim.getTimeStep()
    frames_count = 0
    while (frames_count <= RECORD_FRAMES):
        frames_count += 1

        sim.fill_containers()
        x = sim.getPosition()
        v = sim.getVelocity()
        mass = sim.getMassMatrix()
        rec.record_timestep(x, v)

        A = sim.getEquationMatrix()
        b = sim.getEquationVector()

        delta_v = solve_system(A, b)

        v1 = v + delta_v
        x1 = x + h * v1
        sim.set_state(x1, v1)
        # x2, v2 = newton_iteration(x, v, x1, v1)
        # symulathon.set_state(x2, v2)

        sim.render_state()

    print("Done")
