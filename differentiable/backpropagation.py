import meansquareloss
from solve_system import solve_system


def check_cg_convergence(convergence: int):
    """ Checks weather the conjugate gradient method has converged
    or not and prints a warning with this info."""
    if convergence > 0:
        print(f"Warning: conjugate gradient did not converge\
        with {convergence} iterations")
    elif convergence < 0:
        print("Warning: conjugate gradient illegal input")


class Backpropagation:
    def __init__(self, mass: float, DeltaTime: float):
        # Important quantities
        self.equation_matrix_array = []
        self.dfdp_array = []
        self.dfdx_array = []

        self.mass = mass
        self.h = DeltaTime

        # Loss information
        self.loss = meansquareloss.MeanSquareLoss()
        self.g_array = []
        self.dgdx_array = []
        self.dgdv_array = []

        # Some non essential quantites
        self.x_array = []
        self.v_array = []
        self.f_array = []

    def step(self, x, v, x_t, v_t, equation_matrix, dfdp, dfdx, f=0):
        """ Stores information in forward propagation which is needed
        in backward propagation """
        self.loss.update_containers(x, v, x_t, v_t)

        self.equation_matrix_array.append(equation_matrix)
        self.dfdp_array.append(dfdp)
        self.dfdx_array.append(dfdx)
        self.g_array.append(self.loss.evaluate())
        self.dgdx_array.append(self.loss.get_position_derivative())
        self.dgdv_array.append(self.loss.get_velocity_derivative())
        self.x_array.append(x)
        self.v_array.append(v)
        self.f_array.append(f)

    def get_dgdp(self):
        """ Does the backpropagation to calculate the loss gradient"""
        n_states = len(self.dfdp_array)
        n_steps = n_states - 1
        h = self.h
        dgdx = [a.astype(float) for a in self.dgdx_array]
        dgdv = [a.astype(float) for a in self.dgdv_array]
        S = [0] * n_states

        for i in range(n_steps - 1, -1, -1):
            A = self.equation_matrix_array[i+1]
            eq_vec = (h * dgdx[i+1] + dgdv[i+1])
            sys_solve = solve_system(A, eq_vec)

            dgdx[i] += sys_solve @ (h * self.dfdx_array[i+1]) + dgdx[i+1]
            dgdv[i] += sys_solve @ self.mass
            S[i+1] = sys_solve @ (h*self.dfdp_array[i+1])

        return sum(S)

    def get_g(self):
        """Returns the loss function value of the recorded simulation"""
        return sum(self.g_array)
