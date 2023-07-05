#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np


X = np.load("data/x_data.npy")
Y = np.load("data/y_data.npy")
g = np.load("data/g_data.npy")
maxvalue= 2e6
plt.contourf(X, Y, g, 1000, vmax=maxvalue)
scatter = plt.scatter([1],[1])
arrow = plt.arrow(1, 1, 0.1, 0.1)
arrow.set_data(x=3, y=0.5, dx=0.1, dy=0.1)
help(arrow)
print(type(arrow))
# scatter.set_offsets([1.5,1.5])
plt.colorbar()
plt.show()
