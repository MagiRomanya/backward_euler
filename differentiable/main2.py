#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationReader
from backpropagation import Backpropagation
import symulathon
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm


def newton_iteration(x0, v0, xi, vi):
    A = symulathon.get_equation_matrix()
    dfdx = symulathon.get_force_position_jacobian()
    f = symulathon.get_force()
    # M v0 + h fi + h df/dx (x0 – xi) - (M – h² df/dx) vi
    b = mass * v0 + h * f + h * dfdx @ (x0 - xi) - (mass - h**2 * dfdx) @ vi
    delta_v = solve_system(A, b)
    v1 = vi + delta_v
    x1 = x0 + h * v1
    return x1, v1


def simulate():
    reader = SimulationReader(nDoF)
    backpropagation = Backpropagation(mass, h)
    symulathon.restart_simulation(K_GUESS)
    symulathon.fill_containers()
    for i in range(DIFF_FRAMES+1):
        ##################################
        # Record step for backpropagation
        ##################################
        x = symulathon.get_position()
        v = symulathon.get_velocity()
        x_t, v_t = reader.get_next_state()
        A = symulathon.get_equation_matrix()
        dfdp = symulathon.get_parameter_jacobian()
        dfdx = symulathon.get_force_position_jacobian()
        backpropagation.step(x, v, x_t, v_t, A, dfdp, dfdx)

        ##################################
        # Newton Iterations
        ##################################
        iterations = 2
        xi = x
        vi = v
        for it in range(iterations):
            xi, vi = newton_iteration(x, v, xi, vi)
            symulathon.set_state(xi, vi)
            symulathon.fill_containers()
            # delta_v = vi - v
            # f = symulathon.get_force()
            # print(f"{it+1}th iteration convergence check = {sum(mass @ delta_v - h * f)}")

    dgdp = backpropagation.get_dgdp()
    g = backpropagation.get_g()
    return (g, dgdp)


if __name__ == "__main__":
    symulathon.initialize_scene()
    symulathon.disable_rendering()
    nDoF = symulathon.get_nDoF()
    mass = symulathon.get_mass_matrix()
    h = symulathon.get_time_step()
    K_GUESS = 0.1
    DIFF_FRAMES = 100

    k_values = np.linspace(0.01, 10, 100)
    # np.random.shuffle(k_values)
    # k_values = np.linspace(6.9, 7.1, 20)
    g_values = []
    dgdp_values = []
    for k in tqdm(k_values):
        K_GUESS = k
        g, dgdp = simulate()
        g_values.append(g)
        dgdp_values.append(dgdp[0])

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
    # plt.ylim(-0.2e6, 0.1e6)
    plt.legend()
    plt.xlabel("k value")
    plt.grid()
    plt.show()
    print("Finished succesfully")
