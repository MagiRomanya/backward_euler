#!/usr/bin/env python3
"""Minimize function 2 differentiable parameters."""

from symulathon import Simulation
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from tqdm import tqdm
from simulation_functions import simulate, get_user_wather_graphics

if __name__ == "__main__":
    # Initialize important constant variables
    sim = Simulation(1, 1)
    nDoF = sim.getDoF()
    mass = sim.getMassMatrix()
    h = sim.getTimeStep()
    DIFF_FRAMES = 100

    # Define the region studied

    x0 = np.array([1, 1])
    MAX_ITERATIONS = 1000
    ALPHA = 1
    graphics = get_user_wather_graphics()
    for i in range(MAX_ITERATIONS):
        bp = simulate(x0[0], x0[1], DIFF_FRAMES)
        g = bp.get_g()
        dgdp = bp.get_g()
        x0 = x0 - ALPHA * dgdp
