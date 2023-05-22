#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

finite = np.array([-589, -570, -445, -195])
normal = np.array([-831, -816, -699, -382])
ratio = abs((normal - finite) / finite)
K_init = [0.1, 1, 10, 100]

plt.grid()
plt.xlabel("start K value")
plt.ylabel("Error")
plt.xscale("log")
plt.plot(K_init, ratio)
plt.show()
