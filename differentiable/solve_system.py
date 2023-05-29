#!/usr/bin/env python3

import scipy


def check_cg_convergence(convergence: int):
    """ Checks weather the conjugate gradient method has converged
    or not and prints a warning with this info."""
    if convergence > 0:
        print(f"Warning: conjugate gradient did not converge\
        with {convergence} iterations")
    elif convergence < 0:
        print("Warning: conjugate gradient illegal input")


def solve_system(eq_mat, eq_vec, threshold=1e-8):
    result, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec, tol=threshold)
    check_cg_convergence(convergence)
    return result
