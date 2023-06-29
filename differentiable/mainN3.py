#!/usr/bin/env python3
"""Computes gradient descent for a interpolated parameter cloth."""

from solve_system import solve_system
from recorder import SimulationReader
from backpropagation import Backpropagation
from symulathon import Simulation, count_springs
import sys
import getopt
import numpy as np
from interpolate_values import generate_parameters, get_parameter_indices
from tqdm import tqdm
import matplotlib.pyplot as plt


def newton_iteration(sim: Simulation, x0, v0, xi, vi):
    """Update xi and vi to a new Newton-Rhapson iteration."""
    A = sim.getEquationMatrix()
    dfdx = sim.getForcePositionJacobian()
    f = sim.getForce()
    # M v0 + h fi + h df/dx (x0 – xi) - (M – h² df/dx) vi
    b = mass * v0 + h * f + h * dfdx @ (x0 - xi) - (mass - h**2 * dfdx) @ vi
    delta_v = solve_system(A, b)
    v1 = vi + delta_v
    x1 = x0 + h * v1
    return x1, v1


def get_user_weather_graphics():
    """Return weather the user wants a graphical simulation."""
    enable_graphics = False
    argv = sys.argv[1:]
    opts, args = getopt.getopt(argv, "gp")
    for opt, arg in opts:
        if opt in ['-g']:
            enable_graphics = True
    return enable_graphics


def simulate(k_list: list, k_bend_list: list):
    """Make a simulation with the specified parameters and return the backpropagation information."""
    reader = SimulationReader(nDoF)
    backpropagation = Backpropagation(mass, h)
    sim = Simulation(k_list, k_bend_list, get_user_weather_graphics())
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
        iterations = 1
        xi = x
        vi = v
        for it in range(iterations):
            xi, vi = newton_iteration(sim, x, v, xi, vi)
            sim.set_state(xi, vi)
            sim.fill_containers()

        sim.render_state()
    return backpropagation

def simulation_wrapper(x):
    x = np.clip(x, 0, 500)
    k4 = x[:4]
    k_bend4 = x[4:]
    k_values, k_bend_values = generate_parameters(k4, k_bend4)
    bp = simulate(k_values, k_bend_values)
    g = bp.get_g()
    # print(f"Loss = {g:10.4e}")
    dgdp = bp.get_dgdp()
    dgdp = dgdp[indx]
    return g, dgdp


if __name__ == "__main__":
    # Initialize important constant variables
    sim = Simulation(1, 1)
    nFlex, nBend = count_springs()
    print(f"nFlex = {nFlex}, nBend = {nBend}, total_diff = {nFlex + nBend}")
    nDoF = sim.getDoF()
    mass = sim.getMassMatrix()
    h = sim.getTimeStep()
    DIFF_FRAMES = 100

    # Define inital state
    k_list = np.ones(nFlex) * 10
    k_bend_list = np.ones(nBend) * 0.1
    indx = get_parameter_indices()
    # Minimization process
    MAX_ITER = 1000
    ALPHA = 0.1
    last_loss = 0

    n_points = 40
    k_values = np.linspace(0.5, 5, n_points)
    k_bend_values = np.linspace(1, 500, n_points-1) / 100
    X, Y = np.meshgrid(k_values, k_bend_values)

    g_values = X.tolist()
    ones = np.zeros(X.shape)

    # Computation of the surface and gradients
    for i in tqdm(range(len(k_values))):
        for j in range(len(k_bend_values)):
            k4 = np.ones(4) * k_values[i]
            kbend4 = np.ones(4) * k_bend_values[j]
            k_vec, k_bend_vec = generate_parameters(k4, kbend4)
            bp = simulate(k_vec, k_bend_vec)
            dgdp = bp.get_dgdp()
            g_values[j][i] = bp.get_g()

    # Plotting
    # 3D plot
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    g_values = np.array(g_values)
    print(g_values)

    surf = ax.plot_surface(X, Y, g_values)
    ax.set_zlim(0, np.mean(g_values))
    ax.set(
        xlabel="k values",
        ylabel="k bend values",
        zlabel="loss",
    )

    plt.show()
    plt.contourf(X, Y, g_values, levels=1000, vmax=g_values[-1][-1]/4)
    plt.show()
