#!/usr/bin/env python3

import symulathon
import scipy
import sys
from recorder import SimulationReader
from differentiable import check_cg_convergence
from differentiable import DifferenciableCore


def simulation_step(dif: DifferenciableCore, read: SimulationReader):
    symulathon.process_input()
    symulathon.fill_containers()

    eq_mat = symulathon.get_equation_matrix()
    eq_vec = symulathon.get_equation_vector()
    df_dp = symulathon.get_parameter_jacobian()
    df_dx = symulathon.get_force_position_jacobian()
    mass = symulathon.get_mass_matrix()
    x = symulathon.get_position()
    v = symulathon.get_velocity()
    if (not df_dp.any()):
        print("df/dp is all zeros!")

    x_, v_ = read.get_next_state()  # desired positions and velocities
    dif.step(x, v, x_, v_, eq_mat, df_dx, df_dp, mass)

    # Solve the sparse linear system with conjugate gradient method
    delta_v, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
    check_cg_convergence(convergence)

    symulathon.recieve_delta_v(delta_v)
    symulathon.render_state()


def main():
    symulathon.initialize_scene()
    nDoF = symulathon.get_position().size
    nParameters = symulathon.get_parameter_jacobian().shape[1]
    TimeStep = symulathon.get_time_step()
    print(f"nDoF = {nDoF}")
    print(f"nParameters = {nParameters}")
    print(f"TimeStep = {TimeStep}")

    START_K = 100
    if (len(sys.argv) > 1):
        START_K = float(sys.argv[1])
    dif = DifferenciableCore(4, TimeStep, nDoF, nParameters, start_k=START_K)
    symulathon.restart_simulation(dif.k)

    read = SimulationReader(nDoF)

    while not symulathon.window_should_close():
        simulation_step(dif, read)

        if dif.is_done():
            simulation_step(dif, read)
            dif.update_parameters()
            symulathon.restart_simulation(dif.k)
            read.from_the_start()


if __name__ == "__main__":
    main()
