#!/usr/bin/env python3

import symulathon
import scipy
import numpy as np
import recorder
import differenciable
from differenciable import check_cg_convergence

symulathon.initialize_scene()


def main():
    nDoF = symulathon.get_position().size
    print(f"nDoF = {nDoF}")

    rec = recorder.SimulationRecorder()
    # read = recorder.SimulationReader(nDoF)
    while not symulathon.window_should_close():
        symulathon.process_input()
        symulathon.fill_containers()

        eq_mat = symulathon.get_equation_matrix()
        eq_vec = symulathon.get_equation_vector()
        df_dp = symulathon.get_parameter_jacobian()
        x = symulathon.get_position()
        v = symulathon.get_velocity()
        if (not df_dp.any()): print("df/dp is all zeros!")

        rec.record_timestep(x,v)

        # x_, v_ = read.get_next_state() # desired positions and velocities

        # Solve the sparse linear system with conjugate gradient method
        delta_v, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
        check_cg_convergence(convergence)

        symulathon.recieve_delta_v(delta_v);

        symulathon.render_state()

if __name__ == "__main__":
    main()
