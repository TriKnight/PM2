import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # ✅ REQUIRED for 3D

# Example: helical trajectory
t = np.linspace(0.0, 4.0 * np.pi, 200)
x = np.cos(t)
y = np.sin(t)
z = t / (2 * np.pi)

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")  # 3D axis

ax.plot(x, y, z, label="Helical trajectory", linewidth=2)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z (time)")
ax.set_title("3D Trajectory Example")
ax.legend()

plt.show()
