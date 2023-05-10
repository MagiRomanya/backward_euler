import numpy as np

# g = \sum_i |q_i - q_i*|² with q = x, v -> scalar
# dgdx = 2 (x_i -x_i*) -> nDoF vector
# dgdv = 2 (v_i -v_i*) -> nDoF vector

class MeanSquareLoss:
    def update_containers(self, x : np.array, v : np.array, x_s : np.array, v_s : np.array):
        self.x = x
        self.v = v
        self.x_s = x_s
        self.v_s = v_s


    def evaluate(self) -> float:
        result = 0
        for i in range(len(self.x)):
            result += (self.x[i] - self.x_s[i])**2
            result += (self.v[i] - self.v_s[i])**2
        return result

    def get_position_derivative(self) -> np.array:
        return 2 * ( self.x - self.x_s )

    def get_velocity_derivative(self) -> np.array:
        return 2 * ( self.v - self.v_s )
