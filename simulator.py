#!/usr/bin/env python3

import symulathon
import scipy
import numpy as np

symulathon.initialize_scene()

def check_cg_convergence(convergence : int):
    """ Checks weather the conjugate gradient method has converged or not and prints
    a warning with this info."""
    if convergence > 0:
        print(f"Warning: conjugate gradient did not converge with {convergence} iterations")
    elif convergence < 0:
        print(f"Warning: conjugate gradient illegal input")


def main():
    while not symulathon.window_should_close():
        symulathon.process_input()
        symulathon.fill_containers()

        eq_mat = symulathon.get_equation_matrix()
        eq_vec = symulathon.get_equation_vector()

        # Solve the sparse linear system with conjugate gradient method
        delta_v, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
        check_cg_convergence(convergence)

        symulathon.recieve_delta_v(delta_v);

        symulathon.render_state()

if __name__ == "__main__":
    main()
