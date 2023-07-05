#!/usr/bin/env python3
from symulathon import Simulation, count_springs
from colorama import Fore, Style
from animation_plots import AnimatedPlot
import numpy as np
from simulation_functions import simulate, get_user_plots
from colorama import Fore, Back, Style
from scipy.optimize import minimize, Bounds


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


def simulation_wrapper(x):
    """Recieves x as input k_stretch parameters"""
    bp = simulate(x[:nFlex], x[nFlex:], DIFF_FRAMES)
    g = bp.get_g()
    dgdp = bp.get_dgdp()
    return g, dgdp


if __name__ == "__main__":
    # Initialize important constant variables
    sim = Simulation(1, 1)
    nFlex, nBend = count_springs()
    print(f"nFlex = {nFlex}, nBend = {nBend}, total_diff = {nFlex + nBend}")
    nDoF = sim.getDoF()
    mass = sim.getMassMatrix()
    h = sim.getTimeStep()
    DIFF_FRAMES = 40

    # Define inital state
    k_list = np.ones(nFlex) * 10
    k_bend_list = np.ones(nBend) * 0.1
    parameters = np.concatenate((k_list, k_bend_list))

    # Minimization process
    MAX_ITER = 1000

    draw_plot = get_user_plots()
    if draw_plot:
        plot = AnimatedPlot()
    # Initial optimization for getting near the parameters
    x0 = parameters
    res = minimize(simulation_wrapper, x0, jac=True,
                   bounds=Bounds(0, 600),
                   options={"disp": True,
                            "maxiter": 100,
                            })
    print(res)
    x_result = res.x
    print(x_result)
    stretch_res = x_result[:nFlex]
    bend_res = x_result[nFlex:]
    print(f"STRETCH: mean value = {np.mean(stretch_res)}, var = {np.var(stretch_res)}")
    print(f"BEND: mean value = {np.mean(bend_res)}, var = {np.var(bend_res)}")
    # Real optimization with 100 frames
    DIFF_FRAMES = 100
    x0 = x_result
    res = minimize(simulation_wrapper, x0, jac=True,
                   bounds=Bounds(0, 600),
                   options={"disp": True,})
    print(res)
    x_result = res.x
    print(x_result)
    stretch_res = x_result[:nFlex]
    bend_res = x_result[nFlex:]
    print(f"STRETCH: mean value = {np.mean(stretch_res)}, var = {np.var(stretch_res)}")
    print(f"BEND: mean value = {np.mean(bend_res)}, var = {np.var(bend_res)}")

    # GRADIENT DESCENT
    # previous_g = 0
    # for i in range(MAX_ITER):
    #     k_list, _, g = gradient_descent_iteration(k_list,
    #                                               k_bend_list,
    #                                               DIFF_FRAMES)
    #     diff = g - previous_g
    #     previous_g = g
    #     diff_str = ""
    #     if (diff < 0):
    #         diff_str = Fore.GREEN + str(diff) + Style.RESET_ALL
    #     else:
    #         diff_str = Fore.RED + str(diff) + Style.RESET_ALL

    #     print(f"Iteration #{i}\t loss = {g:10.4e},",
    #           # f"Simulation iterations  = {DIFF_FRAMES}",
    #           "Difference = "+diff_str
    #           )
