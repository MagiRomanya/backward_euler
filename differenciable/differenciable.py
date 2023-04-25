#!/usr/bin/env python3

import meansquareloss
import scipy

def check_cg_convergence(convergence : int):
    """ Checks weather the conjugate gradient method has converged or not and prints
    a warning with this info."""
    if convergence > 0:
        print(f"Warning: conjugate gradient did not converge with {convergence} iterations")
    elif convergence < 0:
        print(f"Warning: conjugate gradient illegal input")

class DifferenciableCore:
    def __init__(self, nDoF, nParameters):
        self.dg_dx = np.zeros(nDoF)
        self.dg_dv = np.zeros(nDoF)
        self.dg_dp = np.zeros(nParameters)
        self.ms_loss = meansquareloss.MeanSquareLoss()


    def step(self, TimeStep, x, v, x_, v_, eq_mat, df_dx, df_dp, mass_matrix):
        self.ms_loss.update_containers(x, v, x_, v_)
        dg_dx_new = self.ms_loss.get_position_derivative()
        dg_dv_new = self.ms_loss.get_velocity_derivative()

        eq_vec = (TimeStep * dg_dx_new + dg_dv_new)

        system_solve = self.solve_system(eq_mat, eq_vec)

        # Update the jacobians
        dg_dx += system_solve * (TimeStep * df_dx) + dg_dx_new
        dg_dv += system_solve * mass_matrix
        dg_dp += system_solve * (TimeStep * df_dp)


    def solve_system(self, eq_mat, eq_vec):
        result , convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
        check_cg_convergence(convergence)
        return result
