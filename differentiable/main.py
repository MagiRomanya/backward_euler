#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationReader
from backpropagation import Backpropagation
import symulathon
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm


def simulate():
    reader = SimulationReader(nDoF)
    backpropagation = Backpropagation(mass, h)
    symulathon.restart_simulation(K_GUESS)
    for i in range(DIFF_FRAMES+1):
        if (symulathon.window_should_close()):
            break
        ##################################
        # ALWAYS fill containers first!
        symulathon.fill_containers()
        ##################################

        x = symulathon.get_position()
        v = symulathon.get_velocity()
        x_t, v_t = reader.get_next_state()

        A = symulathon.get_equation_matrix()
        b = symulathon.get_equation_vector()

        dfdp = symulathon.get_parameter_jacobian()
        dfdx = symulathon.get_force_position_jacobian()
        backpropagation.step(x, v, x_t, v_t, A, dfdp, dfdx)

        delta_v = solve_system(A, b)

        symulathon.recieve_delta_v(delta_v)
        # symulathon.process_input()
        # symulathon.render_state()

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
    DIFF_FRAMES = 2

    k_values = np.linspace(0.01, 2, 1000)
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
    plt.grid()
    plt.show()
    print("Finished succesfully")
