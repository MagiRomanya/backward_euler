#!/usr/bin/env python3

import symulathon
import scipy
import numpy as np
from recorder import SimulationRecorder
from differenciable import check_cg_convergence
from differenciable import DifferenciableCore

def simulation_step(rec : SimulationRecorder):
    symulathon.process_input()
    symulathon.fill_containers()

    eq_mat = symulathon.get_equation_matrix()
    eq_vec = symulathon.get_equation_vector()
    df_dp = symulathon.get_parameter_jacobian()
    df_dx = symulathon.get_force_position_jacobian()
    mass = symulathon.get_mass_matrix()
    x = symulathon.get_position()
    v = symulathon.get_velocity()
    rec.record_timestep(x, v)
    if (not df_dp.any()): print("df/dp is all zeros!")

    # Solve the sparse linear system with conjugate gradient method
    delta_v, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
    check_cg_convergence(convergence)

    symulathon.recieve_delta_v(delta_v);
    symulathon.render_state()


def main():
    N_FRAMES = 1000
    INITIAL_K = 100_000
    symulathon.initialize_scene()
    nDoF = symulathon.get_position().size
    nParameters = symulathon.get_parameter_jacobian().shape[1]
    TimeStep = symulathon.get_time_step()
    print(f"nDoF = {nDoF}")
    print(f"nParameters = {nParameters}")
    print(f"TimeStep = {TimeStep}")

    symulathon.restart_simulation(INITIAL_K)

    rec = SimulationRecorder("teest.csv")
    frames = 0
    while not symulathon.window_should_close():
        simulation_step(rec)
        frames +=1
        if frames >= N_FRAMES:
            break



if __name__ == "__main__":
    main()
