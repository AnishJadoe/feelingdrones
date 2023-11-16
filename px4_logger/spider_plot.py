import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
from pathlib import Path

from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd

from constants import *
from data_loader import get_data_dict




# File path to rosbag
path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/14_11/test_tactile_4'
data_dict = get_data_dict(path)

df_ref = data_dict[TRAJECTORY_SETPOINT]
df_mocap = data_dict[MOCAP]
df_est = data_dict[VEHICLE_ODOMETRY]
df_bar = data_dict[BAR_POSE]
df_sensors = data_dict[TACTILE_DATA]
df_command = data_dict[DRONE_STATE]


time_searching = df_command[(df_command['state'] == SEARCHING)].index
time_grasping = df_command[(df_command['state'] == GRASP)].index
time_evaluating = df_command[(df_command['state'] == EVALUATE)].index
time_touched = df_command[(df_command['state'] == TOUCHED)].index

t_start = min(time_touched)
t_end = min(time_evaluating)
df_est = df_est.loc[t_start:t_end]



def plot_3D_trajectory(ax, df):
    x = df['x']
    y = df['y']
    z = df['z']
    ax.scatter(x, y, z,label='Drone Position', s =0.2)
    ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='red', label='Touch Location')
    return ax

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

plot_3D_trajectory(ax,df_est)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

bar_x = df_bar['y'].iloc[0]
bar_y = df_bar['x'].iloc[0]
bar_z = - df_bar['z'].iloc[0]
# Plot the axis line representing the object
object_center = (bar_x,bar_y ,bar_z )
axis_length = 1  # Adjust the axis length as needed
ax.plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], [object_center[2], object_center[2]], color='blue', label='Object Axis', linewidth=5)
ax.legend()
ax.invert_zaxis()
plt.show()
