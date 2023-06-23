#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationReader
from backpropagation import Backpropagation
from symulathon import Simulation, count_springs
import numpy as np


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


def simulate(k_list: list, k_bend_list: list):
    reader = SimulationReader(nDoF)
    backpropagation = Backpropagation(mass, h)
    sim = Simulation(k_list, k_bend_list, True)
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
    MAX_ITER = 100
    ALPHA = 0.1
    last_loss = 0
    for i in range(MAX_ITER):
        bp = simulate(k_list, k_bend_list)
        g = bp.get_g()
        print(f"Iteration #{i}\t loss = {g:10.4e}, change={g - last_loss:1.4e}")
        last_loss = g

        # Update paramters
        dgdp = bp.get_dgdp()
        parameters -= ALPHA * dgdp
        parameters = np.clip(parameters, 0, 1000000)
        k_list = parameters[0:nFlex]
        k_bend_list = parameters[nFlex:]
