from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import art3d  # Import art3d for adding the cylinder

class FeelyDrone:
    def __init__(self):
        # Initialize necessary variables
        self._beginning = 0
        self._period_counter = 0
        self._obj_pos = {"x": -0.23, "y": 0.58, "z": -1.5}

    def _elipse_searching_event_handler(self, t_trajectory):
        t = t_trajectory
        period = 18
        height_step = 0.1
        x_amplitude = 0.1  # Half of the side length
        y_amplitude = 0.1
        ref_pos = {"x": 0.0, "y": 0.0, "z": 0.0}

        # Define bounds for search trajectory
        x_center = self._obj_pos["x"]
        y_center = self._obj_pos["y"]

        # Calculate continuous trajectory
        x_offset = x_amplitude * np.sin((2 * np.pi / period) * t)
        y_offset = y_amplitude * np.cos((2 * np.pi / period) * t)

        # Set continuous position
        ref_pos["x"] = x_center + x_offset
        ref_pos["y"] = y_center + y_offset
        ref_pos["z"] = self._obj_pos["z"] + 0.2 - height_step * self._period_counter

        self._period_counter = int(t / period)
        return ref_pos

    def _zigzag_searching_event_handler(self, t_trajectory):
        t = t_trajectory
        period = 10
        height_step = -0.05
        x_amplitude = 0.1  # Half of the side length
        y_amplitude = 0.1
        zigzag_frequency = 0.5  # Adjust the zigzag frequency as needed
        zigzag_amplitude = 0.05
        ref_pos = {"x": 0.0, "y": 0.0, "z": 0.0}

        # Define bounds for search trajectory
        x_center = self._obj_pos["x"]
        y_center = self._obj_pos["y"]
        z_center = self._obj_pos["z"] + 0.2

        # Calculate continuous trajectory with zigzag pattern
        x_offset = x_amplitude * np.sin((2 * np.pi / period) * t)
        y_offset = y_amplitude * np.cos((2 * np.pi / period) * t)
        zigzag_offset = zigzag_amplitude * np.sin(2 * np.pi * zigzag_frequency * t)
        z_offset = height_step * self._period_counter  - 0.03* np.sin((1 * np.pi / period) * t)

        # Set continuous position with zigzag pattern
        ref_pos["x"] = x_center + x_offset 
        ref_pos["y"] = y_center + y_offset + zigzag_offset
        ref_pos["z"] = z_center + z_offset

        # Update period counter
        self._period_counter = int(t / period)
        return ref_pos


# Plotting the trajectory
drone = FeelyDrone()

# Simulate the trajectory for a certain time period
time_points = np.arange(0, 100, 0.1)  # Adjust the time range as needed
trajectory_points = []

for t in time_points:
    trajectory_points.append(drone._elipse_searching_event_handler(t))

# Extract x, y, and z coordinates for plotting
x_points = [point["x"] for point in trajectory_points]
y_points = [point["y"] for point in trajectory_points]
z_points = [point["z"] for point in trajectory_points]



cmap = 'viridis'
norm = Normalize(vmin=min(time_points), vmax=max(time_points))
sm = ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])


# Plot the 3D trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_points, y_points, z_points,c=time_points,cmap=cmap, norm=norm, marker='o', s=10, alpha=1, label='Drone Trajectory')
cbar = fig.colorbar(sm, ax=ax, label='Time')
ax.scatter(x_points[0], y_points[0], z_points[0], color='red', label='Starting Point')
ax.scatter(x_points[-1], y_points[-1], z_points[-1], color='green', label='Ending Point')
ax.set_xlabel('X Coordinate')
ax.set_ylabel('Y Coordinate')
ax.set_zlabel('Z Coordinate')

# ax.set_xlim([-1, 1])
# ax.set_ylim([-1, 1])
# ax.set_zlim([-1.5, 0])

# Plot the axis line representing the object
object_center = (drone._obj_pos["x"], drone._obj_pos["y"], drone._obj_pos["z"])
axis_length = 0.2  # Adjust the axis length as needed
ax.plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], [object_center[2], object_center[2]], color='blue', label='Object Axis', linewidth=5)

ax.legend()
plt.show()
