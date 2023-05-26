#!/usr/bin/env python3

import meansquareloss
import numpy as np
import matplotlib.pyplot as plt

DELTA_T = 0.1


class Simulation:
    def __init__(self, k=1):
        # Simulation parameters
        self.delta_t = DELTA_T
        self.iteration = 0

        # Physics parameters
        self.mass = 1
        self.k = 1

        # Inital conditions
        self.x0 = 1
        self.v0 = 0
        self.x = self.x0
        self.v = self.v0

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
        force = - self.k * self.x
        dfdx = -self.k
        dfdp = force / self.k
        A = self.mass - self.delta_t**2 * dfdx

        # Record velocities and positions
        self.x_array.append(self.x)
        self.v_array.append(self.v)

        # Back propagation needed stuff
        if (diff):
            self.equation_mat_array.append(A)
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
        self.v = 1/A*(self.mass * self.v + self.delta_t * force)
        self.x = self.x + self.delta_t*self.v

        self.iteration += 1

    def backpropagation(self):
        dt = self.delta_t
        dgdx = self.dgdx_array
        dgdv = self.dgdv_array
        S = [0] * (self.n_steps+1)

        # print(np.array(dgdx))
        for i in range(self.n_steps - 1, -1, -1):
            eq_vec = (dt * dgdx[i+1] + dgdv[i+1])
            A = self.equation_mat_array[i+1]
            sys_solve = eq_vec / A

            dgdx[i] += sys_solve * (dt * self.dfdx_array[i+1]) + dgdx[i+1]
            dgdv[i] += sys_solve * self.mass
            S[i+1] = sys_solve * dt * self.dfdp_array[i+1]

        return sum(S)

    def start_differentiable(self, n_steps: int, k_ini: float, x_past, v_past):
        # print(f"Differentiable simulation: {len(x_past)} states, {n_steps} steps")
        self.n_steps = n_steps
        self.k = k_ini
        for i in range(n_steps+1):
            self.step(True, x_past, v_past)

        dgdp = self.backpropagation()
        # print(f"Differenciable dgdp = {dgdp}, g = {sum(self.g_array)}\n")
        return dgdp

    def start(self, n_steps: int):
        self.n_steps = n_steps
        self.iteration = 0
        for i in range(n_steps+1):
            self.step()


def analytical_three_states(x_array, v_array, xt_array, vt_array, TimeStep: float, k: float, mass: float):
    """Calculates the loss gradient from the
    analytical hand calculated expression"""

    h = TimeStep
    m = mass
    A = m + h**2*k

    def g(i: int):
        return (x_array[i] - xt_array[i])**2 + (v_array[i] - vt_array[i])**2

    def dgdx(i: int):
        return 2 * (x_array[i] - xt_array[i])

    def dgdv(i: int):
        return 2 * (v_array[i] - vt_array[i])

    state0 = 0

    dv1dp = - h / A * x_array[1]
    dx1dp = h * dv1dp
    state1 = dgdx(1) * dx1dp + dgdv(1) * dv1dp
    if (len(x_array) <= 2):
        return state1

    dv2dp = 1 / A * ((m-h**2*k)*dv1dp - h*x_array[2])
    dx2dp = dx1dp + h * dv2dp
    state2 = dgdx(2) * dx2dp + dgdv(2) * dv2dp

    return (state0 + state1 + state2, g(0) + g(1) + g(2))


def analytical_three_states2(x0, v0, xt_array, vt_array, TimeStep: float, k: float, mass: float, outarray=[]):
    """Calculates the loss gradient from the
    analytical hand calculated expression"""

    h = TimeStep
    m = mass
    A = m + h**2*k
    # First make the two steps of integration
    v1 = 1/A * (m*v0 - h*k*x0)
    x1 = x0 + h*v1

    v2 = 1/A * (m*v1 - h*k*x1)
    x2 = x1 + h*v2
    x_array = [x0, x1, x2]
    v_array = [v0, v1, v2]
    outarray.append(x0)
    outarray.append(x1)
    outarray.append(x2)

    def g(i: int):
        return (x_array[i] - xt_array[i])**2 + (v_array[i] - vt_array[i])**2

    def dgdx(i: int):
        return 2 * (x_array[i] - xt_array[i])

    def dgdv(i: int):
        return 2 * (v_array[i] - vt_array[i])

    state0 = 0

    dv1dp = - h / A * x_array[1]
    dx1dp = h * dv1dp
    state1 = dgdx(1) * dx1dp + dgdv(1) * dv1dp
    if (len(x_array) <= 2):
        return state1

    dv2dp = 1 / A * ((m-h**2*k)*dv1dp - h*x_array[2])
    dx2dp = dx1dp + h * dv2dp
    state2 = dgdx(2) * dx2dp + dgdv(2) * dv2dp

    return (state0 + state1 + state2, g(0) + g(1) + g(2))


STEPS = 2

simulation = Simulation()
simulation.start(STEPS)
diff_simulation = Simulation()
# Ground trouth -> K = 1
GUESS_K = 0.1
dgdp_diff = diff_simulation.start_differentiable(STEPS,
                                                 GUESS_K,
                                                 simulation.x_array,
                                                 simulation.v_array)

diff_simulation2 = Simulation()
delta_k = 0.00001
diff_simulation2.start_differentiable(STEPS,
                                      GUESS_K + delta_k,
                                      simulation.x_array,
                                      simulation.v_array)

g2 = sum(diff_simulation2.g_array)
g1 = sum(diff_simulation.g_array)
dgdp = (g2 - g1) / delta_k
print(f"dgdp differentiable = {dgdp_diff}\n")
# print(f"g1: {g1}, g2 {g2}")
print(f"dgdp finite = {dgdp}\n")

x_array=[]
dgdp, g = analytical_three_states2(diff_simulation.x_array[0],
                                   diff_simulation.v_array[0],
                                   simulation.x_array,
                                   simulation.v_array,
                                   TimeStep=DELTA_T,
                                   k=GUESS_K,
                                   mass=1,
                                   outarray=x_array)

print(f"x_array analytical = {x_array}")
print(f"x_array diff = {diff_simulation.x_array}")
print(f"Analytical gradient: {dgdp}")

#####################################################
##################### PLOTING #######################
#####################################################
gs = []
gs2 = []
dgdp_diff = []
dgdp_analytical = []
ks = np.linspace(0.01, 2, 300)
for k in ks:
    s = Simulation()
    dgdp = s.start_differentiable(STEPS, k, simulation.x_array, simulation.v_array)
    gs.append(sum(s.g_array))
    dgdp_diff.append(dgdp)
    dgdp, g2 = analytical_three_states2(s.x_array[0],
                                        s.v_array[0],
                                        simulation.x_array,
                                        simulation.v_array,
                                        TimeStep=DELTA_T,
                                        k=k,
                                        mass=1)
    gs2.append(g2)
    dgdp_analytical.append(dgdp)

dgdp_finite = []
for i in range(len(gs) - 1):
    dgdp_finite.append((gs[i+1] - gs[i]) / (ks[i+1] - ks[i]))
dgdp_finite.append(dgdp_finite[-1])

dgdp_diff = np.array(dgdp_diff)

plt.plot(ks, gs, label="Loss function")
plt.plot(ks, gs2, label="Loss function 2")
plt.plot(ks, dgdp_diff, label="Diff dgdp")
plt.plot(ks, dgdp_analytical,"x", label="Analytincal dgdp")
plt.plot(ks, dgdp_finite, label="Finite dgdp")
plt.xlabel("K parameter")
plt.legend()
plt.grid()
plt.show()
