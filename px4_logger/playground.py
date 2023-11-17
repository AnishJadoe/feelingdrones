import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from constants import *
from data_loader import get_data_dict

def get_trajectories(df,trajectories, trajectory_start):
    
    
    df_trajectory = df[trajectory_start:]
    previous_timestamp = trajectory_start
    
    if df_trajectory['state'].loc[trajectory_start] == LAND:
        return trajectories
    
    for current_timestamp in df_trajectory.index:
        if (df_trajectory.loc[current_timestamp]['state'] - df_trajectory.loc[previous_timestamp]['state']) == (EVALUATE - GRASP):
            trajectory_end = previous_timestamp
            trajectories.append((trajectory_start,trajectory_end))
            break
        previous_timestamp = current_timestamp
    trajectories = get_trajectories(df_trajectory, trajectories, current_timestamp)
    
    return trajectories
    
path = "/home/anish/Documents/Thesis/Drone/ros2_bag_files/succesfull_perches/test_tactile_success_1/"
data_dict = get_data_dict(path)

df_ref = data_dict[TRAJECTORY_SETPOINT]
df_mocap = data_dict[MOCAP]
df_est = data_dict[VEHICLE_ODOMETRY]
df_bar = data_dict[BAR_POSE]
df_sensors = data_dict[TACTILE_DATA]
df_command = data_dict[DRONE_STATE]
trajectories = []
trajectory_start = min(df_command[df_command['state'] == TOUCHED].index)
get_trajectories(df_command, trajectories, trajectory_start)
