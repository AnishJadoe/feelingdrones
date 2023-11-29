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
        period = 10
        height_step = 0.1
        x_amplitude = 0.1  # Half of the side length
        y_amplitude = 0.2
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
        height_step = -0.1
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
        z_offset = height_step * self._period_counter # - 0.03* np.sin((1 * np.pi / period) * t)

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
time_points = np.arange(0, 50, 0.1)  # Adjust the time range as needed
elipse_trajectory_points = []
zigzag_trajectory_points = []

for t in time_points:
    elipse_trajectory_points.append(drone._elipse_searching_event_handler(t))
    zigzag_trajectory_points.append(drone._zigzag_searching_event_handler(t))

# Extract x, y, and z coordinates for plotting
x_points_ellipse = [point["x"] for point in elipse_trajectory_points]
y_points_ellipse = [point["y"] for point in elipse_trajectory_points]
z_points_ellipse = [point["z"] for point in elipse_trajectory_points]

x_points_zigzag = [point["x"] for point in zigzag_trajectory_points]
y_points_zigzag = [point["y"] for point in zigzag_trajectory_points]
z_points_zigzag = [point["z"] for point in zigzag_trajectory_points]

cmap = 'viridis'
norm = Normalize(vmin=min(time_points), vmax=max(time_points))
sm = ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])

# Create a single figure with two subplots
fig, axes = plt.subplots(1, 2, figsize=(12, 5), subplot_kw={'projection': '3d'})
# Plot the axis line representing the object
object_center = (drone._obj_pos["x"], drone._obj_pos["y"], drone._obj_pos["z"])
axis_length = 0.2  # Adjust the axis length as needed

# Plot the 3D trajectory for ellipse
axes[0].scatter(x_points_ellipse, y_points_ellipse, z_points_ellipse, c=time_points, cmap=cmap, norm=norm, marker='o', s=10, alpha=1, label='Drone Trajectory')
axes[0].scatter(x_points_ellipse[0], y_points_ellipse[0], z_points_ellipse[0], color='red', label='Starting Point', s=70)
axes[0].scatter(x_points_ellipse[-1], y_points_ellipse[-1], z_points_ellipse[-1], color='green', label='Ending Point', s=70)
axes[0].set_xlabel('X [m]')
axes[0].set_ylabel('Y [m]')
axes[0].set_zlabel('Z [m]')
axes[0].plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], [object_center[2], object_center[2]], color='blue', label='Object Axis', linewidth=5)
axes[0].invert_zaxis()
axes[0].view_init(elev=30, azim=60, roll=0)
axes[0].set_xticks(np.linspace(min(x_points_ellipse), max(x_points_ellipse), 5))
axes[0].set_yticks(np.linspace(min(y_points_ellipse), max(y_points_ellipse), 5))
axes[0].set_zticks(np.linspace(min(z_points_ellipse), max(z_points_ellipse), 5))
axes[0].legend(loc='upper right')
axes[0].set_title("Ellipse trajectory around perching object")

# Plot the 3D trajectory for zigzag
axes[1].scatter(x_points_zigzag, y_points_zigzag, z_points_zigzag, c=time_points, cmap=cmap, norm=norm, marker='o', s=10, alpha=1, label='Drone Trajectory')
axes[1].scatter(x_points_zigzag[0], y_points_zigzag[0], z_points_zigzag[0], color='red', label='Starting Point', s=70)
axes[1].scatter(x_points_zigzag[-1], y_points_zigzag[-1], z_points_zigzag[-1], color='green', label='Ending Point', s=70)
axes[1].set_xlabel('X [m]')
axes[1].set_ylabel('Y [m]')
axes[1].set_zlabel('Z [m]')
axes[1].plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], [object_center[2], object_center[2]], color='blue', label='Object Axis', linewidth=5)
axes[1].invert_zaxis()
axes[1].view_init(elev=30, azim=60, roll=0)
axes[1].set_xticks(np.linspace(min(x_points_ellipse), max(x_points_ellipse), 5))
axes[1].set_yticks(np.linspace(min(y_points_ellipse), max(y_points_ellipse), 5))
axes[1].set_zticks(np.linspace(min(z_points_ellipse), max(z_points_ellipse), 5))
axes[1].legend(loc='upper right')
axes[1].set_title("Zigzag trajectory around perching object")

plt.savefig('/home/anish/Documents/Thesis/Plots/trajectories.pdf', format='pdf')
