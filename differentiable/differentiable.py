#!/usr/bin/env python3

import meansquareloss
import scipy
import numpy as np


def check_cg_convergence(convergence: int):
    """ Checks weather the conjugate gradient method has converged
    or not and prints a warning with this info."""
    if convergence > 0:
        print(f"Warning: conjugate gradient did not converge\
        with {convergence} iterations")
    elif convergence < 0:
        print("Warning: conjugate gradient illegal input")


class DifferenciableCore:
    def __init__(self, total_steps, TimeStep,  nDoF, nParameters, start_k=100):
        self.nDoF = nDoF
        self.nParameters = nParameters
        self.dg_dx = []
        self.dg_dv = []
        self.df_dx = []
        self.df_dp = []
        self.eq_mat = []
        self.ms_loss = meansquareloss.MeanSquareLoss()
        self.TimeStep = TimeStep
        self.k = start_k
        self.loss_list = []
        self.framecount = 0
        self.TOTAL_STEPS = total_steps
        self.iterations = 0
        self.mass_matrix = 0

    def is_done(self):
        self.framecount += 1
        if self.TOTAL_STEPS == self.framecount:
            self.framecount = 0
            self.iterations += 1
            return True

        return False

    def step(self, x, v, x_, v_, eq_mat, df_dx, df_dp, mass_matrix):
        self.ms_loss.update_containers(x, v, x_, v_)
        dg_dx_new = self.ms_loss.get_position_derivative()
        dg_dv_new = self.ms_loss.get_velocity_derivative()

        self.mass_matrix = mass_matrix

        self.dg_dx.append(dg_dx_new)
        self.dg_dv.append(dg_dv_new)
        self.df_dx.append(df_dx)
        self.df_dp.append(df_dp)
        self.eq_mat.append(eq_mat)

        self.loss_list.append(self.ms_loss.evaluate())

    def solve_system(self, eq_mat, eq_vec):
        result, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
        # print(f"Equation matrix shape = {eq_mat.shape},\
        # Equation vector shape = {eq_vec.shape}")
        check_cg_convergence(convergence)
        return result

    def backpropagation(self):
        TimeStep = self.TimeStep
        # self.dg_dx -> partial derivatives
        # dg_dx -> total derivatives
        dg_dx = self.dg_dx
        dg_dv = self.dg_dv
        dg_dp = np.zeros(self.nParameters)
        S = [0] * (self.TOTAL_STEPS+1)

        for i in range(self.TOTAL_STEPS - 1, -1, -1):
            eq_vec = (TimeStep * dg_dx[i+1] + dg_dv[i+1])
            sys_solve = self.solve_system(self.eq_mat[i+1], eq_vec)  # i+1

            dg_dx[i] += sys_solve * (TimeStep * self.df_dx[i+1]) + dg_dx[i+1]
            dg_dv[i] += sys_solve * self.mass_matrix
            S[i+1] = np.matmul(sys_solve, (TimeStep * self.df_dp[i+1]))
            # dg_dp    += np.matmul(sys_solve , (TimeStep * self.df_dp[i+1]))

        # The loop does not cover the S_0 case
        eq_vec = (TimeStep * dg_dx[0] + dg_dv[0])
        sys_solve = self.solve_system(self.eq_mat[0], eq_vec)
        S[0] = np.matmul(sys_solve, (TimeStep * self.df_dp[0]))

        dg_dp = sum(S)
        # print(self.df_dp[0].T[0])
        return dg_dp

    def update_parameters(self):
        dg_dp = self.backpropagation()
        current_k = self.k

        # Gradient descent
        g_value = sum(self.loss_list)

        ALPHA = 100
        self.k -= ALPHA*dg_dp[0]

        if (self.k < 0):
            print("WARNING: Parameter K was prevented from being negative.")
            self.k = 1

        print(f"New k = {self.k:.2f},"
              f" loss = {g_value:.3f}, "
              f"dgdp = {dg_dp[0]:.8f}, "
              f"Real error: {(100_000 - current_k):.2f}")

        # Reset all derivatives to zero
        self.dg_dx = []
        self.dg_dv = []
        self.df_dx = []
        self.df_dp = []
        self.eq_mat = []
        self.loss_list = []
