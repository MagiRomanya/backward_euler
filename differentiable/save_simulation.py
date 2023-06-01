#!/usr/bin/env python3

from solve_system import solve_system
from recorder import SimulationRecorder
import symulathon

def newton_step():
    symulathon.fill_containers()
    A = symulathon.get_equation_matrix()
    b = symulathon.get_equation_vector()

    delta_v = solve_system(A, b)

    v2 = v + delta_v
    x2 = x + h * v2

    symulathon.set_state(x2, v2)


if __name__ == "__main__":
    rec = SimulationRecorder()
    symulathon.initialize_scene()
    K_VALUE = 1
    RECORD_FRAMES = 300
    symulathon.restart_simulation(K_VALUE)
    h = symulathon.get_time_step()
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

        v1 = v + delta_v
        x1 = x + h * v1
        symulathon.set_state(x1, v1)

        # newton_step()

        symulathon.process_input()
        symulathon.render_state()

    print("Done")
