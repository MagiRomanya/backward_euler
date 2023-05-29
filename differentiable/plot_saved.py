#!/usr/bin/env python3

from recorder import SimulationReader
import matplotlib.pyplot as plt

nDoF = 6
reader = SimulationReader(nDoF)

history = reader.get_all_history()

plt.plot(history[3])
plt.show()
