#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# SoC (%), Temperature (°C), and their respective limits
soc_range = np.array([0, 80, 90, 95, 98, 100])
temp_range = np.array([-20, -10, 0, 10, 25, 40])
# Max regen limits (A) at nominal conditions (temp = 25°C)
soc_limit = np.array([100, 100, 80, 50, 10, 0])
# Max regen limits (A) at nominal SoC = 80%
temp_limit = np.array([0, 10, 30, 60, 100, 90])

# Function to compute SoC-based limit
def soc_regen_limit(soc):
    return np.interp(soc, soc_range, soc_limit)

# Function to compute temperature-based limit
def temp_regen_limit(temp):
    return np.interp(temp, temp_range, temp_limit)

# Combined limiter: lowest of SoC and temperature limits
def regen_current_limit(soc, temp):
    return min(soc_regen_limit(soc), temp_regen_limit(temp))

# Simulate a 2D grid of regen limits
soc_vals = np.linspace(0, 100, 101)
temp_vals = np.linspace(-20, 40, 61)
regen_matrix = np.array([[regen_current_limit(s, t) for s in soc_vals] for t in temp_vals])

# Plotting the 2D regen limit surface
plt.figure(figsize=(10, 6))
contour = plt.contourf(soc_vals, temp_vals, regen_matrix, levels=20, cmap='plasma')
plt.title('Regen Current Limit vs. SoC and Temperature')
plt.xlabel('State of Charge (%)')
plt.ylabel('Cell Temperature (°C)')
cbar = plt.colorbar(contour)
cbar.set_label('Max Regen Current Limit (A)')
plt.grid(True)
plt.tight_layout()
plt.show()

