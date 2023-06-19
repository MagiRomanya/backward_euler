#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationReader
from backpropagation import Backpropagation
from symulathon import Simulation
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from tqdm import tqdm


def newton_iteration(sim: Simulation, x0, v0, xi, vi):
    A = sim.getEquationMatrix()
    dfdx = sim.getForcePositionJacobian()
    f = sim.getForce()
    # M v0 + h fi + h df/dx (x0 – xi) - (M – h² df/dx) vi
    b = mass * v0 + h * f + h * dfdx @ (x0 - xi) - (mass - h**2 * dfdx) @ vi
    delta_v = solve_system(A, b)
    v1 = vi + delta_v
    x1 = x0 + h * v1
    return x1, v1


def simulate(k, k_bend):
    reader = SimulationReader(nDoF)
    backpropagation = Backpropagation(mass, h)
    sim = Simulation([k, k_bend])
    sim.fill_containers()
    for i in range(DIFF_FRAMES+1):
        ##################################
        # Record step for backpropagation
        ##################################
        x = sim.getPosition()
        v = sim.getVelocity()
        x_t, v_t = reader.get_next_state()
        A = sim.getEquationMatrix()
        dfdp = sim.getParameterJacobian()
        dfdx = sim.getForcePositionJacobian()
        backpropagation.step(x, v, x_t, v_t, A, dfdp, dfdx)

        ##################################
        # Newton Iterations
        ##################################
        iterations = 2
        xi = x
        vi = v
        for it in range(iterations):
            xi, vi = newton_iteration(sim, x, v, xi, vi)
            sim.set_state(xi, vi)
            sim.fill_containers()
            # delta_v = vi - v
            # f = symulathon.get_force()
            # print(f"{it+1}th iteration convergence check = {sum(mass @ delta_v - h * f)}")

    return backpropagation


# def calc_gradient(surface: np.array):
#     dgdx = np.zeros(surface.shape)
#     dgdy = np.zeros(surface.shape)
#     x_dim, y_dim = surface.shape
#     for i in range(x_dim):
#         for j in range(y_dim):
#             dgdx =

if __name__ == "__main__":
    sim = Simulation([1, 1])
    nDoF = sim.getDoF()
    mass = sim.getMassMatrix()
    h = sim.getTimeStep()
    DIFF_FRAMES = 10

    n_points = 30
    k_values = np.linspace(0.01, 10, n_points)
    k_bend_values = np.linspace(-10, 500, n_points-1) / 100
    X, Y = np.meshgrid(k_values, k_bend_values)
    g_values = X.tolist()
    dgdk_values = X.tolist()
    dgdk_bend_values = X.tolist()
    ones = np.zeros(X.shape)
    for i in tqdm(range(len(k_values))):
        for j in range(len(k_bend_values)):
            bp = simulate(k_values[i], k_bend_values[j])
            dgdp = bp.get_dgdp()
            dgdk_values[j][i] = dgdp[0]
            dgdk_bend_values[j][i] = dgdp[1]
            g_values[j][i] = bp.get_g()

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    g_values = np.array(g_values)
    dgdk_values = np.array(dgdk_values)
    dgdk_bend_values = np.array(dgdk_bend_values)
    dk = k_values[1]-k_values[0]
    dk_bend = k_bend_values[1]-k_bend_values[0]
    dgdk_bend_values_finite, dgdk_values_finite = np.gradient(g_values, dk, dk_bend)
    diff = np.sqrt(
        (dgdk_values - dgdk_values_finite)**2 +
        (dgdk_bend_values - dgdk_bend_values_finite)**2)
    dfdp_magnitude_finite = np.sqrt(
        dgdk_values_finite**2 + dgdk_bend_values_finite**2)
    diff_rel = diff / dfdp_magnitude_finite

    colors = cm.Greys(diff)
    surf = ax.plot_surface(X, Y, g_values, facecolors=colors)

    gradient = ax.quiver(X, Y, g_values, dgdk_values, dgdk_bend_values, ones,
                         arrow_length_ratio=0.001,
                         length=0.5,
                         normalize=True)

    gradient_finite = ax.quiver(X, Y, g_values,
                                dgdk_values_finite,
                                dgdk_bend_values_finite,
                                ones,
                                arrow_length_ratio=0.001,
                                color="red",
                                length=0.5,
                                normalize=True)
    ax.set(
        xlabel="k values",
        ylabel="k bend values",
        zlabel="loss",
    )
    # Add a color bar which maps values to colors.
    # fig.colorbar(surf, shrink=0.5, aspect=5)
    plt.show()
    plt.contourf(X, Y, g_values, levels=40)
    plt.colorbar()
    plt.quiver(X, Y, dgdk_values, dgdk_bend_values, color="blue")
    plt.quiver(X, Y, dgdk_values_finite, dgdk_bend_values_finite , color="red")
    plt.show()
