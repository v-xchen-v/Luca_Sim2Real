import os, sys
module_path = os.path.abspath(os.path.join('.'))
if module_path not in sys.path:
    sys.path.append(module_path)
    
from coordinates.frames import coordinate_frames
import matplotlib.pyplot as plt
from pytransform3d.transformations import plot_transform

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot all frames
for name, T in coordinate_frames.items():
    plot_transform(ax=ax, A2B=T, name=name, s=0.3)

# Customize plot limits and labels
ax.set_xlim([-1, 3])
ax.set_ylim([-1, 3])
ax.set_zlim([-1, 3])
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
plt.title("Coordinate Frames Visualization")

# Show plot
plt.show()