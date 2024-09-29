import numpy as np
import matplotlib.pyplot as plt

# Load the stored data
optimal_x = np.loadtxt("optimal_x_receding.csv", delimiter=",")
optimal_u = np.loadtxt("optimal_u_receding.csv", delimiter=",")

# Extract position, velocity, and input data
x_m = optimal_x[:, 0]
y_m = optimal_x[:, 1]
v_x = optimal_x[:, 2]
v_y = optimal_x[:, 3]
delta_vx = optimal_u[:, 0]
delta_vy = optimal_u[:, 1]

# Time vector
time = np.arange(optimal_u.shape[0]) * 0.1  # T = 0.1

# Plot 1: Position evolution in the environment
plt.figure(figsize=(10, 6))
plt.plot(x_m, y_m, 'bo-', label="Position (x_m, y_m)")
plt.xlim([0, 60])
plt.ylim([0, 60])
plt.axvline(x=30, color='k', linestyle='-', label="Cell Boundary (x=30)")
plt.axhline(y=30, color='k', linestyle='-', label="Cell Boundary (y=30)")
plt.title("Position Evolution in the Environment")
plt.xlabel("x_m (meters)")
plt.ylabel("y_m (meters)")
plt.grid(True)
plt.legend()
plt.show()

# Plot 2: Control Inputs over Time
fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Subplot for delta_vx
axs[0].step(time, delta_vx, where='post', label="Delta v_x", color='blue')
axs[0].set_ylabel("Delta v_x (m/s^2)")
axs[0].grid(True)
axs[0].legend()

# Subplot for delta_vy
axs[1].step(time, delta_vy, where='post', label="Delta v_y", color='red')
axs[1].set_ylabel("Delta v_y (m/s^2)")
axs[1].set_xlabel("Time (seconds)")
axs[1].grid(True)
axs[1].legend()

plt.suptitle("Control Inputs over Time")
plt.show()

# Plot 3: Velocities over Time
fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# Subplot for v_x
axs[0].step(time, v_x, where='post', label="v_x", color='green')  # Exclude the last point to match time vector length
axs[0].set_ylabel("v_x (m/s)")
axs[0].grid(True)
axs[0].legend()

# Subplot for v_y
axs[1].step(time, v_y, where='post', label="v_y", color='purple')  # Exclude the last point to match time vector length
axs[1].set_ylabel("v_y (m/s)")
axs[1].set_xlabel("Time (seconds)")
axs[1].grid(True)
axs[1].legend()

plt.suptitle("Velocities over Time")
plt.show()
