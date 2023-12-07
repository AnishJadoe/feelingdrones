from matplotlib.patches import Patch
import numpy as np
import matplotlib.pyplot as plt
import os
from constants import *
from common import get_trajectories
from data_loader import get_data_dict
import pandas as pd
from matplotlib import cm


offset_mapping = {BOT_PHALANGE_1:np.array([X1,Y1, Z]),
                BOT_PHALANGE_2:np.array([0,-Y1, Z]),
                BOT_PHALANGE_3:np.array([-X1,Y1, Z]),
                MID_PHALANGE_1:np.array([X1,Y2, Z]),
                MID_PHALANGE_2:np.array([0, -Y2, Z]),
                MID_PHALANGE_3:np.array([-X1,Y2, Z]),
                TOP_PHALANGE_1:np.array([X1,Y3, Z]),
                TOP_PHALANGE_2:np.array([0, -Y3, Z]),
                TOP_PHALANGE_3:np.array([-X1,Y3,Z]) 
                }

def plot_horizontal_cylinder(ax, radius=0.5, height=0, center=(0, 0, 0), num_points=500):

    theta = np.linspace(0, 2*np.pi, num_points)
    x = np.linspace(center[0] - 0.5*height, center[0] + 0.5*height, num_points)

    theta, x = np.meshgrid(theta, x)
    y = radius * np.sin(theta) + center[1]
    z = radius * np.cos(theta) + center[2]

    # Plot the surface of the cylinder
    ax.plot_surface(x, y, z, color='grey', alpha=1)

    
def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

def plot_2D_trajectory(ax, df_touched,df_grasp):
    x_touched = df_touched['x'] 
    y_touched = df_touched['y'] 
    x_grasp = df_grasp['x']
    y_grasp = df_grasp['y']
    ax.plot([x_touched,x_grasp], [y_touched,y_grasp], color='black')
    ax.scatter(x_touched, y_touched,color='red')
    ax.scatter(x_grasp, y_grasp, color='green')
    return ax

def plot_3D_trajectory(ax, df):
    x = df['x'] 
    y = df['y'] 
    z = df['z']
    ax.plot(x, y, z, '--')
    ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='red')
    ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='green')
    return ax

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# File path to rosbag
folder_path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/offset_05'
path = 'test_tactile_25'

data_dict = get_data_dict(f'{folder_path}/{path}')

df_ref = data_dict[TRAJECTORY_SETPOINT]
df_mocap = data_dict[MOCAP]
df_est = data_dict[VEHICLE_ODOMETRY]
df_bar = data_dict[BAR_POSE]
df_sensors = data_dict[TACTILE_DATA]
df_command = data_dict[DRONE_STATE]


trajectories = []
trajectory_start = min(df_command[df_command['state'] == TOUCHED].index)
trajectories = get_trajectories(df_command, trajectories, trajectory_start)[1:]


# sensor_touched = int(df_sensors_touch[df_sensors_touch == 1].index[0][-1])
# for trajectory in trajectories:
#     t_touch = trajectory[0]
#     t_grasp = trajectory[1]

#     sensor_threshold = df_sensors.iloc[0]
#     df_touch = df_est.loc[:t_touch].iloc[-1]
#     df_sensors_touch = df_sensors.loc[:t_touch].iloc[-1]
#     df_grasp = df_est.loc[:t_grasp].iloc[-1]

#     plot_2D_trajectory(ax,df_touch, df_grasp)
        
for trajectory in trajectories:
    t_touch = trajectory[0]
    t_grasp = trajectory[1]
    
    plot_3D_trajectory(ax,df_est.loc[t_touch:t_grasp])
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('Tactile controller reference positions after touch event')
bar_x = df_bar['x'].iloc[0]
bar_y = df_bar['y'].iloc[0]
bar_z = df_bar['z'].iloc[0]

plot_horizontal_cylinder(ax, radius=0.05, height=3, center=(bar_x,bar_y, bar_z))
color_mappnig_matrix = {'Grasp Position': 'green',
                        'Touch Position':'red'}
legend_handles = [Patch(color=color, label=label) for label,color in color_mappnig_matrix.items()]

## 2D
# object_center = (bar_x,bar_y)
# axis_length = 1  # Adjust the axis length as needed
# ax.plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], color='grey', linewidth=5, label='Perching Object')
# ax.legend(handles=legend_handles)
# plt.show()
# 3D
object_center = (bar_x,bar_y ,bar_z )
axis_length = 1  # Adjust the axis length as needed
# ax.plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], [object_center[2], object_center[2]], color='blue', label='Object Axis', linewidth=5)
ax.legend()
ax.set_ylim(-0.8,-0.5)
ax.set_xlim(-1,1)
ax.set_zlim(-1.6,-1.3)
ax.invert_zaxis()
plt.show()
# plt.savefig('/home/anish/Documents/Thesis/Plots/spider_plot.pdf', format='pdf')
