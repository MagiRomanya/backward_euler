#!/usr/bin/env python3
from solve_system import solve_system
from recorder import SimulationReader
from backpropagation import Backpropagation
from symulathon import Simulation, count_springs
import sys
import getopt
from colorama import Fore, Style
from animation_plots import AnimatedPlot
import numpy as np


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


def get_user_plots():
    """Return weather the user wants a plot of the gradient descent convergence."""
    enable_plots = False
    argv = sys.argv[1:]
    opts, args = getopt.getopt(argv, "gp")
    for opt, arg in opts:
        if opt in ['-p']:
            enable_plots = True
    return enable_plots


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
        iterations = 3
        xi = x
        vi = v
        for it in range(iterations):
            xi, vi = newton_iteration(sim, x, v, xi, vi)
            sim.set_state(xi, vi)
            sim.fill_containers()

        sim.render_state()
    return backpropagation


if __name__ == "__main__":
    # Initialize important constant variables
    sim = Simulation(1, 1)
    nFlex, nBend = count_springs()
    print(f"nFlex = {nFlex}, nBend = {nBend}, total_diff = {nFlex + nBend}")
    nDoF = sim.getDoF()
    mass = sim.getMassMatrix()
    h = sim.getTimeStep()
    DIFF_FRAMES = 100

    # CHECK weather the paramters indices make sense
    # k_list = np.arange(0, 2*nFlex, 2)
    # k_bend_list = np.arange(1, 2*nBend+1, 2)
    # parameters = np.concatenate((k_list, k_bend_list))
    # sim = Simulation(k_list, k_bend_list)
    # p = sim.getDiffParameters()
    # print(f"Error : {parameters - p}, total {sum(parameters - p)}")

    # Define inital state
    k_list = np.ones(nFlex) * 10
    k_bend_list = np.ones(nBend) * 0.1
    parameters = np.concatenate((k_list, k_bend_list))

    # Minimization process
    MAX_ITER = 1000
    ALPHA = 0.00005
    last_loss = 0

    plot = AnimatedPlot()
    draw_plot = get_user_plots()
    # GRADIENT DESCENT
    for i in range(MAX_ITER):
        bp = simulate(k_list, k_bend_list)
        g = bp.get_g()
        if (draw_plot):
            plot.get_data(g)
        dg = g - last_loss
        color = ""
        if (dg > 0):
            color = Fore.RED
        else:
            color = Fore.GREEN

        print(f"Iteration #{i}\t loss = {g:10.4e}, " +
              color +
              f"change={dg:1.4e}" +
              Style.RESET_ALL)
        last_loss = g

        # Update paramters
        dgdp = bp.get_dgdp()
        parameters -= ALPHA * dgdp
        parameters = np.clip(parameters, 0, 1000)
        k_list = parameters[0:nFlex]
        k_bend_list = parameters[nFlex:]
