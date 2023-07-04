#!/usr/bin/env python3
from symulathon import Simulation, count_springs
from colorama import Fore, Style
from animation_plots import AnimatedPlot
import numpy as np
from simulation_functions import simulate, get_user_plots
from colorama import Fore, Back, Style


def gradient_descent_iteration(k_list, k_bend_list, DIFF_FRAMES):
    parameters = np.concatenate((k_list, k_bend_list))
    bp = simulate(k_list, k_bend_list, DIFF_FRAMES)
    g = bp.get_g()
    if (draw_plot):
        plot.get_data(g)


    # Update paramters
    dgdp = bp.get_dgdp()
    parameters -= ALPHA * dgdp
    parameters = np.clip(parameters, 0, 1000)
    return parameters[0:nFlex], parameters[nFlex:], g


if __name__ == "__main__":
    counter = 0
    # Initialize important constant variables
    sim = Simulation(1, 1)
    nFlex, nBend = count_springs()
    print(f"nFlex = {nFlex}, nBend = {nBend}, total_diff = {nFlex + nBend}")
    nDoF = sim.getDoF()
    mass = sim.getMassMatrix()
    h = sim.getTimeStep()
    DIFF_FRAMES = 200

    # Define inital state
    k_list = np.ones(nFlex) * 10
    k_bend_list = np.ones(nBend) * 0.1
    parameters = np.concatenate((k_list, k_bend_list))

    # Minimization process
    MAX_ITER = 1000
    ALPHA = 0.005

    plot = AnimatedPlot()
    draw_plot = get_user_plots()

    previous_g = 0
    # GRADIENT DESCENT
    for i in range(MAX_ITER):
        k_list, _, g = gradient_descent_iteration(k_list,
                                                  k_bend_list,
                                                  DIFF_FRAMES)
        diff = g - previous_g
        previous_g = g
        diff_str = ""
        if (diff < 0):
            diff_str = Fore.GREEN + str(diff) + Style.RESET_ALL
        else:
            diff_str = Fore.RED + str(diff) + Style.RESET_ALL

        print(f"Iteration #{i}\t loss = {g:10.4e},",
              # f"Simulation iterations  = {DIFF_FRAMES}",
              "Difference = "+diff_str
              )
