#!/usr/bin/env python3

import symulathon
import scipy
import numpy as np

symulathon.initialize_scene()

while not symulathon.window_should_close():
    symulathon.process_input()
    symulathon.fill_containers()

    eq_mat = symulathon.get_equation_matrix()
    eq_vec = symulathon.get_equation_vector().ravel()

    # print("Equation vector shape", eq_vec.shape)

    # print("Equation vector")
    # print(eq_vec)
    # print("Equation vector type", type(eq_vec))

    delta_v = scipy.sparse.linalg.cg(eq_mat, eq_vec)[0]
    # print("delta v type ", type(delta_v))
    # print("dleta v shape", delta_v.shape)
    # print(delta_v)

    symulathon.recieve_delta_v(delta_v);
    symulathon.render_state()
