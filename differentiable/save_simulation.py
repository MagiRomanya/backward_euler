#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationRecorder
import symulathon


if __name__ == "__main__":
    rec = SimulationRecorder()
    symulathon.initialize_scene()
    K_VALUE = 1
    RECORD_FRAMES = 200
    symulathon.restart_simulation(K_VALUE)

    frames_count = 0
    while (not symulathon.window_should_close()) and (frames_count <= RECORD_FRAMES):
        frames_count += 1

        symulathon.fill_containers()
        x = symulathon.get_position()
        v = symulathon.get_velocity()
        rec.record_timestep(x, v)

        A = symulathon.get_equation_matrix()
        b = symulathon.get_equation_vector()

        delta_v = solve_system(A, b)

        symulathon.recieve_delta_v(delta_v)
        symulathon.process_input()
        symulathon.render_state()

    print("Done")
