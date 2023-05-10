#!/usr/bin/env python3

import symulathon
import scipy
import numpy as np
from recorder import SimulationReader
from differenciable import check_cg_convergence
from differenciable import DifferenciableCore
import meansquareloss

def simulation_step(dif : DifferenciableCore, read : SimulationReader):
    symulathon.process_input()
    symulathon.fill_containers()

    eq_mat = symulathon.get_equation_matrix()
    eq_vec = symulathon.get_equation_vector()
    df_dp = symulathon.get_parameter_jacobian()
    df_dx = symulathon.get_force_position_jacobian()
    mass = symulathon.get_mass_matrix()
    x = symulathon.get_position()
    v = symulathon.get_velocity()
    if (not df_dp.any()): print("df/dp is all zeros!")

    x_, v_ = read.get_next_state() # desired positions and velocities
    dif.step(x, v, x_, v_, eq_mat, df_dx, df_dp, mass)

    # Solve the sparse linear system with conjugate gradient method
    delta_v, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
    check_cg_convergence(convergence)

    symulathon.recieve_delta_v(delta_v);
    symulathon.render_state()

def main():
    symulathon.initialize_scene()
    ndof = symulathon.get_position().size
    nparameters = symulathon.get_parameter_jacobian().shape[1]
    timestep = symulathon.get_time_step()
    print(f"ndof = {ndof}")
    print(f"nparameters = {nparameters}")
    print(f"timestep = {timestep}")

    dif = differenciablecore(100, timestep, ndof, nparameters);
    symulathon.restart_simulation(dif.k)

    read = simulationreader(ndof)

    while not symulathon.window_should_close():
        simulation_step(dif, read)

        if dif.is_done():
            symulathon.restart_simulation(dif.k)
            read.from_the_start()

def simulation_step_finite(read : SimulationReader) -> float:
    TimeStep = symulathon.get_time_step()

    symulathon.process_input()
    symulathon.fill_containers()

    eq_mat = symulathon.get_equation_matrix()
    eq_vec = symulathon.get_equation_vector()
    x = symulathon.get_position()
    v = symulathon.get_velocity()

    x_, v_ = read.get_next_state() # desired positions and velocities

    # Solve the sparse linear system with conjugate gradient method
    delta_v, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
    check_cg_convergence(convergence)

    symulathon.recieve_delta_v(delta_v);
    symulathon.render_state()

    loss = meansquareloss.MeanSquareLoss()
    loss.update_containers(x, v, x_, v_)
    return loss.evaluate()

def main_finite():
    symulathon.initialize_scene()
    ndof = symulathon.get_position().size
    nparameters = symulathon.get_parameter_jacobian().shape[1]
    timestep = symulathon.get_time_step()
    print(f"ndof = {ndof}")
    print(f"nparameters = {nparameters}")
    print(f"timestep = {timestep}")


    read = SimulationReader(ndof)
    FRAMES = 100
    current_frame = 0
    g_value = 0
    g_value_dp = 0
    k_value = 1000 # initial k
    dp = 0.1
    is_second_simulation = False
    symulathon.restart_simulation(k_value)
    while not symulathon.window_should_close():
        current_frame+=1
        if not is_second_simulation:
            g_value += simulation_step_finite(read)
            if current_frame >= FRAMES:
                current_frame = 0
                symulathon.restart_simulation(k_value + dp)
                read.from_the_start()
                is_second_simulation = True

        else:
            g_value_dp += simulation_step_finite(read)
            if current_frame >= FRAMES:
                current_frame = 0
                dg_dp = (g_value_dp - g_value) / dp
                print(f"dg = {g_value_dp - g_value}, dg_dp = {dg_dp}")

                g_value = 0
                g_value_dp = 0

                ALPHA = 10000
                k_value = k_value - ALPHA * dg_dp
                print(f"New k={k_value}")
                symulathon.restart_simulation(k_value)
                read.from_the_start()
                is_second_simulation = False


if __name__ == "__main__":
    main_finite()
