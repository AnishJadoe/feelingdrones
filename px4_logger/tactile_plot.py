import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as ColorMapper
from matplotlib.patches import Polygon 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
from pathlib import Path
import pandas as pd
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from constants import *
from data_loader import get_data_dict

IDLE = 0  
HOVER = 1
MOVING = 2
TOUCHED = 3
GRASP = 4
EVALUATE = 5
REFINE = 6
SEARCHING =7


sensor_name_mapping = {
        6:'Bottom Phalange 1', 
        7:'Middle Phalange 1',
        8:'Top Phalange 1',
        3:'Bottom Phalange 2',
        4:'Middle Phalange 2',
        5:'Top Phalange 2',
        0:'Bottom Phalange 3',
        1:'Middle Phalange 3',
        2:'Top Phalange 3',
        9:'EMPTY',
        10:'EMPTY',
        11:'EMPTY'
    }


# File path to rosbag
path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/14_11/test_tactile_4'
data_dict = get_data_dict(path)

df_ref = data_dict[TRAJECTORY_SETPOINT]
df_mocap = data_dict[MOCAP]
df_est = data_dict[VEHICLE_ODOMETRY]
df_bar = data_dict[BAR_POSE]
df_sensors = data_dict[TACTILE_DATA]
df_command = data_dict[DRONE_STATE]

searching = df_command[df_command['state'] == SEARCHING].index
touched = df_command[df_command['state'] == TOUCHED].index
grasping = df_command[df_command['state'] == GRASP].index
evaluating = df_command[df_command['state'] == EVALUATE].index

t_start = min(df_ref.index)
t_end = max(df_ref.index)

df_ref = df_ref.loc[t_start:t_end]
df_mocap = df_mocap.loc[t_start:t_end]
df_est = df_est.loc[t_start:t_end]
df_bar = df_bar.loc[t_start:t_end]
df_sensors = df_sensors.loc[t_start:t_end]
df_command = df_command.loc[t_start:t_end]



axes = ['x','y','z']
fig,ax = plt.subplots(4,1)
for i,axes in enumerate(axes):
    ax[i].plot(df_ref.index,df_ref[axes], label='reference')
    ax[i].plot(df_est.index,df_est[axes], label='estimated odometry')
    ax[i].plot(df_mocap.index,df_mocap[axes], label='mocap')
    ax[i].hlines(df_bar[axes].iloc[0],xmin=min(df_bar.index),xmax=max(df_bar.index),linestyles='dashed', label='bar')
    ax[i].set_title(axes.capitalize())


df_sensors = df_sensors.mask(df_sensors - df_sensors.iloc[0] <= -10, 1)
df_sensors = df_sensors.mask(df_sensors > 1, 0)


for i in range(12):
        if (df_sensors[f'sensor_{i}'].sum() > 1):
            ax[3].plot(df_sensors.index,
                    df_sensors[f'sensor_{i}'],
                    label=sensor_name_mapping[i]
                    )
            
cmap = ColorMapper.get_cmap('Pastel2')
for i in range(4):
    ax[i].set_xlim(t_start,t_end)
    if len(searching):
        ax[i].axvspan(min(searching), max(searching), facecolor=cmap.colors[0], alpha=0.5, label='Searching')
    if len(touched):
        ax[i].axvspan(min(touched), max(touched), facecolor=cmap.colors[1], alpha=0.5, label='Touched')
    if len(grasping):
        ax[i].axvspan(min(grasping), max(grasping), facecolor=cmap.colors[2], alpha=0.5, label='Grasping')
    if len(evaluating):
        ax[i].axvspan(min(evaluating), max(evaluating), facecolor=cmap.colors[3], alpha=0.5, label='Evaluating')
    ax[i].grid()
    ax[i].legend()

ax[2].invert_yaxis()
plt.show()
