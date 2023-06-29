#!/usr/bin/env python3
import numpy as np
from symulathon import Simulation
from interpolate_values import generate_parameters

k = 4
k_bend = 0
sim1 = Simulation(k, k_bend)
spring_pp_indx = sim1.getSpringIndices()
bend_spring_pp_indx = sim1.getBendSpringIndices()

k, k_bend = generate_parameters(np.ones(4)*4, np.ones(4)*0)
sim2 = Simulation(k, k_bend)

nFlex = int(len(spring_pp_indx) / 2)
nBend = int(len(bend_spring_pp_indx) / 2)

parameters = np.zeros(nFlex + nBend)
parameters[:nFlex] = 4
parameters[nFlex:] = 0
print(parameters)
print(k+ k_bend)
