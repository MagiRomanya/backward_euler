#!/usr/bin/env python3
"""Plot loss landscape for 2 differentiable parameters."""

from symulathon import Simulation
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from tqdm import tqdm
from simulation_functions import simulate

if __name__ == "__main__":
    # Initialize important constant variables
    sim = Simulation(1, 1)
    nDoF = sim.getDoF()
    mass = sim.getMassMatrix()
    h = sim.getTimeStep()
    DIFF_FRAMES = 100

    # Define the region studied
    n_points = 11
    # k_values = np.linspace(0.01, 10, n_points)
    # k_bend_values = np.linspace(-10, 500, n_points-1) / 100
    k_values = np.linspace(3.95, 4.05, n_points)
    k_bend_values = np.linspace(0, 0.2, 51)
    X, Y = np.meshgrid(k_values, k_bend_values)

    g_values = X.tolist()
    dgdk_values = X.tolist()
    dgdk_bend_values = X.tolist()
    ones = np.zeros(X.shape)

    # Computation of the surface and gradients
    for i in tqdm(range(len(k_values))):
        for j in range(len(k_bend_values)):
            bp = simulate(k_values[i], k_bend_values[j], DIFF_FRAMES)
            dgdp = bp.get_dgdp()
            dgdk_values[j][i] = dgdp[0]
            dgdk_bend_values[j][i] = dgdp[1]
            g_values[j][i] = bp.get_g()

    # Finite differences
    dk = k_values[1]-k_values[0]
    dk_bend = k_bend_values[1]-k_bend_values[0]
    dgdk_bend_values_finite, dgdk_values_finite = np.gradient(g_values, dk_bend, dk)

    # Plotting
    # 3D plot
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    g_values = np.array(g_values)
    dgdk_values = np.array(dgdk_values)
    dgdk_bend_values = np.array(dgdk_bend_values)
    diff = np.sqrt(
        (dgdk_values - dgdk_values_finite)**2 +
        (dgdk_bend_values - dgdk_bend_values_finite)**2)
    dfdp_magnitude_finite = np.sqrt(
        dgdk_values_finite**2 + dgdk_bend_values_finite**2)
    diff_rel = diff / np.max(dfdp_magnitude_finite)

    colors = cm.Greys(diff)
    # surf = ax.plot_surface(X, Y, g_values, facecolors=colors)
    surf = ax.plot_surface(X, Y, g_values)

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

    plt.contourf(X, Y, diff_rel, levels=40)
    plt.colorbar()
    plt.quiver(X, Y, dgdk_values, dgdk_bend_values, color="blue")
    plt.quiver(X, Y, dgdk_values_finite, dgdk_bend_values_finite , color="red")
    plt.show()
