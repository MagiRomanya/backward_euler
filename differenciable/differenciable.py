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
    def __init__(self, TimeStep,  nDoF, nParameters):
        self.nDoF = nDoF;
        self.nParameters = nParameters;
        self.dg_dx = np.zeros((1,nDoF))
        self.dg_dv = np.zeros((1,nDoF))
        self.dg_dp = np.zeros((1,nParameters))
        self.ms_loss = meansquareloss.MeanSquareLoss()
        self.TimeStep = TimeStep
        self.k = 10000
        self.loss_list = []

    def step(self, x, v, x_, v_, eq_mat, df_dx, df_dp, mass_matrix):
        TimeStep = self.TimeStep
        self.ms_loss.update_containers(x, v, x_, v_)
        dg_dx_new = self.ms_loss.get_position_derivative()
        dg_dv_new = self.ms_loss.get_velocity_derivative()

        eq_vec = (TimeStep * dg_dx_new + dg_dv_new)

        system_solve = self.solve_system(eq_mat, eq_vec).reshape(1, -1) # to row vector


        # Update the jacobians
        # REVIEW: la matriz mass y df_dx son sparse, el operador * puede que no esté haciendo lo que deberia
        self.dg_dx += system_solve * (TimeStep * df_dx) + dg_dx_new.reshape(1,-1)
        self.dg_dv += system_solve * mass_matrix
        self.dg_dp += np.matmul(system_solve , (TimeStep * df_dp))

        self.loss_list.append(self.ms_loss.evaluate())

    def solve_system(self, eq_mat, eq_vec):
        result , convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
        check_cg_convergence(convergence)
        return result

    def update_parameters(self):
        # Newton step
        # self.k -= self.ms_loss.evaluate() / self.dg_dp[0][0]
        self.k -= 100000*self.dg_dp[0][0]

        print(f"New k = {self.k}, Error = {sum(self.loss_list) / len(self.loss_list)}")

        # Reset all derivatives to zero
        self.dg_dx = np.zeros((1, self.nDoF))
        self.dg_dv = np.zeros((1, self.nDoF))
        self.dg_dp = np.zeros((1, self.nParameters))
        self.loss_list = []
