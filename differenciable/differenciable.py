#!/usr/bin/env python3

import meansquareloss
import scipy
import numpy as np

def check_cg_convergence(convergence : int):
    """ Checks weather the conjugate gradient method has converged or not and prints
    a warning with this info."""
    if convergence > 0:
        print(f"Warning: conjugate gradient did not converge with {convergence} iterations")
    elif convergence < 0:
        print(f"Warning: conjugate gradient illegal input")

class DifferenciableCore:
    def __init__(self, total_steps, TimeStep,  nDoF, nParameters):
        self.nDoF = nDoF;
        self.nParameters = nParameters;
        self.dg_dx = []
        self.dg_dv = []
        self.df_dx = []
        self.df_dp = []
        self.eq_mat = []
        self.ms_loss = meansquareloss.MeanSquareLoss()
        self.TimeStep = TimeStep
        self.k = 1000 # initial k
        self.loss_list = []
        self.framecount = 0
        self.TOTAL_STEPS = total_steps
        self.iterations = 0
        self.mass_matrix = 0

    def is_done(self):
        self.framecount+=1;
        if self.TOTAL_STEPS == self.framecount:
            self.framecount = 0
            self.iterations+=1
            self.update_parameters()
            print(f"Iteration : {self.iterations}")
            return True

        return False

    def step(self, x, v, x_, v_, eq_mat, df_dx, df_dp, mass_matrix):
        TimeStep = self.TimeStep
        self.ms_loss.update_containers(x, v, x_, v_)
        dg_dx_new = self.ms_loss.get_position_derivative()
        dg_dv_new = self.ms_loss.get_velocity_derivative()

        # eq_vec = (TimeStep * dg_dx_new + dg_dv_new)

        # system_solve = self.solve_system(eq_mat, eq_vec).reshape(1, -1) # to row vector

        self.mass_matrix = mass_matrix;

        # Update the jacobians
        # REVIEW: la matriz mass y df_dx son sparse, el operador * puede que no esté haciendo lo que deberia
        # self.dg_dx.append( system_solve * (TimeStep * df_dx) + dg_dx_new.reshape(1,-1) )
        # self.dg_dv.append( system_solve * mass_matrix )
        # self.dg_dp.append( np.matmul(system_solve , (TimeStep * df_dp)) )

        self.dg_dx.append(dg_dx_new)
        self.dg_dv.append(dg_dv_new)
        self.df_dx.append(df_dx)
        self.df_dp.append(df_dp)
        self.eq_mat.append(eq_mat)

        self.loss_list.append(self.ms_loss.evaluate())

    def solve_system(self, eq_mat, eq_vec):
        result , convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
        check_cg_convergence(convergence)
        return result

    def backpropagation(self):
        dg_dx = self.dg_dx[self.TOTAL_STEPS -1]
        dg_dv = self.dg_dv[self.TOTAL_STEPS -1]
        dg_dp = np.zeros(self.nParameters)

        for j in range(self.TOTAL_STEPS - 1):
            i = self.TOTAL_STEPS - j - 2;
            eq_vec = (self.TimeStep * dg_dx + dg_dv)
            # eq_vec = (self.TimeStep * self.dg_dx[i+1] + self.dg_dv[i+1])

            sys_solve = self.solve_system(self.eq_mat[i+1], eq_vec) # i+1

            dg_dx = self.dg_dx[i] + (self.TimeStep * self.df_dx[i+1]).dot(sys_solve) + dg_dx
            dg_dv = self.dg_dv[i] + self.mass_matrix.dot(sys_solve)
            # dg_dx = dg_dx + (self.TimeStep * self.df_dx[i+1]).dot(sys_solve) + dg_dx
            # dg_dv = dg_dv + self.mass_matrix.dot(sys_solve)
            dg_dp += np.matmul(sys_solve , (self.TimeStep * self.df_dp[i+1]))
            # print(f"sys shape: {sys_solve.shape}, dg_dx shape : {dg_dx.shape}, dg_dv shape : {dg_dv.shape}, dg_dp shape : {dg_dp.shape}")


        return dg_dp

    def update_parameters(self):
        dg_dp = self.backpropagation()
        current_k = self.k
        # Newton step
        g_value = sum(self.loss_list)
        # self.k -= g_value / self.dg_dp[0][0]
        # self.k -= self.ms_loss.evaluate() / self.dg_dp[0][0]

        self.k -= 10000*dg_dp[0]
        # self.k -= 1*dg_dp[0][0]

        self.k = max(self.k, 1)

        print(f"New k = {self.k}, Error = {g_value}, Real error: {(100_000 - current_k) / 100_000}")

        # Reset all derivatives to zero
        self.dg_dx = []
        self.dg_dv = []
        self.df_dx = []
        self.df_dp = []
        self.eq_mat = []
        self.loss_list = []
