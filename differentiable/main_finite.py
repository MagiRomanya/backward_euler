#!/usr/bin/env python3

import symulathon
import scipy
import sys
from recorder import SimulationReader
from differentiable import check_cg_convergence
from meansquareloss import MeanSquareLoss


def simulation_step(read: SimulationReader):
    symulathon.process_input()
    symulathon.fill_containers()

    eq_mat = symulathon.get_equation_matrix()
    eq_vec = symulathon.get_equation_vector()
    x = symulathon.get_position()
    v = symulathon.get_velocity()

    x_, v_ = read.get_next_state()  # desired positions and velocities

    # Solve the sparse linear system with conjugate gradient method
    delta_v, convergence = scipy.sparse.linalg.cg(eq_mat, eq_vec)
    check_cg_convergence(convergence)

    symulathon.recieve_delta_v(delta_v)
    symulathon.render_state()

    loss = MeanSquareLoss()
    loss.update_containers(x, v, x_, v_)
    return loss.evaluate()


def main_finite():
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

    print(START_K)

    TOTAL_FRAMES = 4
    frames = 0
    is_second = False
    current_k = START_K
    delta_k = 0.001

    symulathon.restart_simulation(current_k)

    read = SimulationReader(nDoF)
    counter = 0
    g = 0
    dg = 0
    while not symulathon.window_should_close():
        frames += 1
        if not is_second:
            g += simulation_step(read)
            counter += 1
            if TOTAL_FRAMES < frames:
                # print(f"Counter = {counter}")
                counter = 0
                is_second = True
                frames = 0
                symulathon.restart_simulation(current_k + delta_k)
                read.from_the_start()

        else:
            dg += simulation_step(read)
            if TOTAL_FRAMES < frames:
                is_second = False
                frames = 0
                dgdp = (dg - g) / delta_k
                ALPHA = 1000
                current_k -= ALPHA * dgdp

                print(f"Current k = {current_k:.2f}, "
                      f"loss = {g:.3f}, "
                      f"dgdp = {dgdp:.8f}")

                read.from_the_start()
                symulathon.restart_simulation(current_k)
                dg = 0
                g = 0


if __name__ == "__main__":
    main_finite()
