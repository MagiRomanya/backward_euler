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


def simulate(k, k_bend):
    reader = SimulationReader(nDoF)
    backpropagation = Backpropagation(mass, h)
    symulathon.restart_simulation([k, k_bend])
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

    return backpropagation


if __name__ == "__main__":
    symulathon.initialize_scene(False)
    nDoF = symulathon.get_nDoF()
    mass = symulathon.get_mass_matrix()
    h = symulathon.get_time_step()
    DIFF_FRAMES = 10

    n_points = 20
    k_values = np.linspace(0.01, 10, n_points)
    k_bend_values = np.linspace(-10, 500, n_points - 5) / 100
    X, Y = np.meshgrid(k_values, k_bend_values)
    g_values = X.tolist()
    dgdp_values = X.tolist()
    for i in tqdm(range(len(k_values))):
        for j in range(len(k_bend_values)):
            bp = simulate(k_values[i], k_bend_values[j])
            dgdp_values[j][i] = bp.get_dgdp()
            g_values[j][i] = bp.get_g()

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    g_values = np.array(g_values)
    surf = ax.plot_surface(X, Y, g_values)
    ax.set(
        xlabel="k values",
        ylabel="k bend values",
        zlabel="loss",
    )
    plt.show()
    symulathon.terminate()
