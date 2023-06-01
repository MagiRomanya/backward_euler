#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import symulathon
from solve_system import solve_system
from recorder import SimulationReader
from backpropagation import Backpropagation


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


def newton_iterations(x0, v0, n=3):
    A = symulathon.get_equation_matrix()
    b = symulathon.get_equation_vector()

    delta_v = solve_system(A, b)

    v1 = v0 + delta_v
    x1 = x0 + h * v1
    symulathon.set_state(x1, v1)
    symulathon.fill_containers()
    f = symulathon.get_force()
    # print(f"1st iteration check = {(mass @ delta_v - h * f)[3:]}")

    for i in range(n-1):
        x1, v1 = newton_iteration(x0, v0, x1, v1)


def simulate():
    reader = SimulationReader(nDoF)
    backpropagation = Backpropagation(mass, h)
    symulathon.restart_simulation(K_GUESS)
    for _ in range(DIFF_FRAMES+1):
        ##################################
        # ALWAYS fill containers first!
        symulathon.fill_containers()
        ##################################

        x = symulathon.get_position()
        v = symulathon.get_velocity()
        x_t, v_t = reader.get_next_state()

        A = symulathon.get_equation_matrix()

        dfdp = symulathon.get_parameter_jacobian()
        dfdx = symulathon.get_force_position_jacobian()
        backpropagation.step(x, v, x_t, v_t, A, dfdp, dfdx)

        newton_iterations(x, v)
        # symulathon.process_input()
        # symulathon.render_state()

    return backpropagation


if __name__ == "__main__":
    symulathon.initialize_scene()
    symulathon.disable_rendering()
    nDoF = symulathon.get_nDoF()
    mass = symulathon.get_mass_matrix()
    h = symulathon.get_time_step()
    K_GUESS = 0.1
    DIFF_FRAMES = 100

    k_values = np.linspace(0.01, 10, 1000)
    # k_values = [0.1,0.2]
    g_values = []
    dgdp_values = []
    for k in tqdm(k_values):
        K_GUESS = k
        bp = simulate()
        g_values.append(bp.get_g())
        dgdp_values.append(bp.get_dgdp()[0])
        # plt.plot(bp.x_array)
        # plt.show()

    # Calculate finite differences
    dgdp_finite = []
    for i in range(len(g_values)-1):
        value = (g_values[i+1]-g_values[i]) / (k_values[i+1] - k_values[i])
        dgdp_finite.append(value)
    dgdp_finite.append(dgdp_finite[-1])

    print(len(k_values), len(dgdp_finite), len(g_values), len(dgdp_values))
    plt.plot(k_values, g_values, "-", label="Loss Function")
    plt.plot(k_values, dgdp_finite, ".", label="Finite dgdp")
    plt.plot(k_values, dgdp_values, "x", label="Backpropagation dgdp")
    # plt.ylim(-0.1e5, 0.1e5)
    print(sum(np.array(dgdp_finite) - np.array(dgdp_values)) / len(dgdp_finite))
    plt.legend()
    plt.xlabel("K values")
    plt.grid()
    plt.show()
    print("Finished succesfully")
