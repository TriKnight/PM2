import matplotlib.pyplot as plt

# Example: visualize a simple trajectory
times = [0.0, 0.1, 0.2, 0.3, 0.4]
positions = [0.0, 0.05, 0.1, 0.15, 0.2]
commands = [0.0, 0.1, 0.2, 0.2, 0.2]

plt.figure(figsize=(8, 4))

# Plot command (dashed) and measured position (solid)
plt.plot(times, commands, "--", label="Command position [rad]")
plt.plot(times, positions, label="Measured position [rad]")

plt.xlabel("Time [s]")
plt.ylabel("Position [rad]")
plt.title("Motor Position: Command vs Measurement")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig("trajectory_plot.png")
plt.show()