#!/usr/bin/env python3
#
import meansquareloss
import numpy as np


class Simulation:
    def __init__(self, k=1):
        # Simulation parameters
        self.delta_t = 0.05
        self.iteration = 0

        # Physics parameters
        self.mass = 1
        self.k = 1
        self.L = 0

        # Inital conditions
        self.x = 1
        self.v = 0

        # Recording path
        self.x_array = []
        self.v_array = []

        # Differentiable simulation
        self.dfdp_array = []
        self.dfdx_array = []
        self.dgdx_array = []
        self.dgdv_array = []
        self.g_array = []
        self.equation_mat_array = []
        self.loss = meansquareloss.MeanSquareLoss()

    def step(self, diff=False, x_past=[], v_past=[]):
        # Calculate force & derivative
        force = - self.k * (self.x - self.L)
        dfdx = self.k * self.L
        dfdp = force / self.k

        # Record velocities and positions
        self.x_array.append(self.x)
        self.v_array.append(self.v)

        # Back propagation needed stuff
        if (diff):
            self.equation_mat_array.append(self.mass - self.delta_t**2 * dfdx)
            self.dfdp_array.append(dfdp)
            self.dfdx_array.append(dfdx)
            self.loss.update_containers(np.array([self.x]),
                                        np.array([self.v]),
                                        np.array([x_past[self.iteration]]),
                                        np.array([v_past[self.iteration]]))

            self.g_array.append(self.loss.evaluate())
            self.dgdx_array.append(self.loss.get_position_derivative()[0])
            self.dgdv_array.append(self.loss.get_velocity_derivative()[0])

        # Integration (implicit euler)
        self.v = 1/(self.mass - self.delta_t**2 * dfdx)*(self.mass * self.v + self.delta_t * force)
        self.x = self.x + self.v

        self.iteration += 1

    def backpropagation(self):
        dt = self.delta_t
        dgdx = self.dgdx_array
        dgdv = self.dgdv_array
        S = [0] * (self.n_steps+1)

        # print(np.array(dgdx))
        for i in range(self.n_steps - 1, -1, -1):
            eq_vec = (dt * dgdx[i+1] + dgdv[i+1])
            sys_solve = 1 / self.equation_mat_array[i] * eq_vec

            dgdx[i] += sys_solve * (dt * self.dfdx_array[i+1]) + dgdx[i+1]
            dgdv[i] += sys_solve * self.mass
            S[i+1] = sys_solve * dt * self.dfdp_array[i+1]

        eq_vec = (dt * dgdx[0] + dgdv[0])
        sys_solve = 1 / self.equation_mat_array[0] * eq_vec
        S[0] = sys_solve * dt * self.dfdp_array[0]
        return sum(S)

    def start_differentiable(self, n_steps: int, k_ini: float, x_past, v_past):
        print(f"Differentiable simulation: {len(x_past)} states, {n_steps} steps")
        self.n_steps = n_steps
        self.k = k_ini
        for i in range(n_steps+1):
            self.step(True, x_past, v_past)

        dgdp = self.backpropagation()
        print(f"Differenciable dgdp = {dgdp}\n")

    def start(self, n_steps: int):
        self.n_steps = n_steps
        self.iteration = 0
        for i in range(n_steps+1):
            self.step()


if __name__ == "__main__":
    simulation = Simulation()
    simulation.start(100)
    diff_simulation = Simulation()
    # Ground trouth -> K = 1
    GUESS_K = 0.1
    diff_simulation.start_differentiable(100,
                                         GUESS_K,
                                         simulation.x_array,
                                         simulation.v_array)

    diff_simulation2 = Simulation()
    delta_k = 0.00001
    diff_simulation2.start_differentiable(100,
                                          GUESS_K + delta_k,
                                          simulation.x_array,
                                          simulation.v_array)

    g2 = sum(diff_simulation2.g_array)
    g1 = sum(diff_simulation.g_array)
    dgdp = (g2 - g1) / delta_k
    print(f"Loss1: {g1}, Loss2 {g2}")
    print(f"dgdp finite = {dgdp}\n")
