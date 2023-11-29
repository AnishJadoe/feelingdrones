from matplotlib.patches import Patch
import numpy as np
import matplotlib.pyplot as plt
import os
from constants import *
from data_loader import get_data_dict


def plot_2D_trajectory(ax, df):
    x = df['x']
    y = df['y']
    ax.plot([x.iloc[0],x.iloc[-1]], [y.iloc[0],y.iloc[-1]], color='black')
    ax.scatter(x.iloc[0], y.iloc[0],color='red')
    ax.scatter(x.iloc[-1], y.iloc[-1], color='green')
    return ax


# File path to rosbag
folder_path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/17_11'


fig,ax = plt.subplots()
for path in os.listdir(folder_path):
    data_dict = get_data_dict(f'{folder_path}/{path}')

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

    if time_touched.empty or time_evaluating.empty:
        continue
    t_start = min(time_touched)
    t_end = min(time_evaluating)
    df_est = df_est.loc[t_start:t_end]
    plot_2D_trajectory(ax,df_est)
    

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_title('Tactile controller reference positions after touch event')
bar_x = df_bar['x'].iloc[0]
bar_y = df_bar['y'].iloc[0]


color_mappnig_matrix = {'Grasp Position': 'green',
                        'Touch Position':'red'}
legend_handles = [Patch(color=color, label=label) for label,color in color_mappnig_matrix.items()]

# Plot the axis line representing the object
object_center = (bar_x,bar_y)
axis_length = 1  # Adjust the axis length as needed
ax.plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], color='blue', linewidth=5, label='Perching Object')
ax.legend(handles=legend_handles)
# plt.show()
# plt.savefig('/home/anish/Documents/Thesis/Plots/spider_plot.pdf', format='pdf')
