from matplotlib import font_manager
import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from data_loader import get_data_dict
from constants import *
from collections import Counter

# Specify the font file path
font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'

# Register the font
font_manager.fontManager.addfont(font_path)

plt.rcParams['font.size'] = 35
plt.rcParams['font.family'] = 'Times New Roman'
fig_size = (20,10)
dpi = 250

def get_tries(df):
    # Initialize variables
    grasp_attempts = 0
    in_grasp_state = False

    # Iterate through the DataFrame
    for _, row in df.iterrows():
        current_state = row['state']

        # Check if the drone is in the GRASP state
        if current_state == 4:
            in_grasp_state = True

        # Check if the drone transitions from GRASP to EVALUATE or from EVALUATE back to GRASP
        elif in_grasp_state and current_state in [5, 8]:
            grasp_attempts += 1
            in_grasp_state = False
        
    return grasp_attempts

def get_aggregates(folder_paths,name,n_max=8):
    print(f"Getting aggregates for {name}")
    n_plots = 0
    n_succes = 0
    n_fail = 0
    time_before_land = []
    tries_before_land = []
    for folder_path in folder_paths:
        for path in os.listdir(HOME+folder_path):
            if n_plots >= n_max:
                break
            data_dict = get_data_dict(f'{HOME+folder_path}/{path}')
            df_command = data_dict[DRONE_STATE]
            df_sensors = data_dict[TACTILE_DATA]
            time_evaluating = df_command[(df_command['state'] == EVALUATE)].index
            time_touched = df_command[(df_command['state'] == TOUCHED)].index
            time_landing = df_command[(df_command['state'] == LAND)].index
            if time_evaluating.empty or df_sensors.empty or time_touched.empty:
                continue
            try:
                df_sensors = df_sensors.mask(df_sensors - df_sensors.iloc[0] <= -15, 1)
                df_sensors = df_sensors.mask(df_sensors > 1, 0)
                landing_tactile_state = np.array(df_sensors.loc[max(time_evaluating):].iloc[0])
            except:
                continue
            stable_grasp = ((
                (sum(landing_tactile_state - np.array([0,1,0,0,1,0,0,1,0,0,0,0])) >= 0) \
                or (sum(landing_tactile_state - np.array([1,0,0,1,0,0,1,0,0,0,0,0])) >= 0 )
                ) \
                and not (sum(landing_tactile_state) > 9)) or not time_landing.empty  # every sensor on
            
            if stable_grasp:
                print(f"{path} SUCCES ")
                t_start = min(time_touched)
                t_end = max(time_evaluating)
                time_before_land.append(t_end-t_start)
                tries_before_land.append(get_tries(df_command))
                n_plots += 1
                n_succes += 1
            else:
                print(f"{path} FAILED")
                tries_before_land.append(-1)
                n_fail += 1
                # n_plots += 1
                
    print(f'Used {n_succes + n_fail} trials')
    print(f'Found {n_succes} succesfull and {n_fail} failed perches')
        
    return time_before_land,tries_before_land
    
# File path to rosbag
HOME = '/home/anish/Documents/Thesis/Drone/ros2_bag_files/'
elipse_paths = ['closed_loop_tactile_6_12', 'closed_loop_tactile_7_12']
zigzag_paths =  ['zigzag_7_12']
#folder_paths = ['closed_loop_tactile_7_12']
searching_trajectories = ['Elipse searching trajectory', 'Zigzag searching trajectory']


fig,ax = plt.subplots(figsize=fig_size)

elipse_aggrigate_time,elipse_paths_aggrigate_tries = get_aggregates(folder_paths=elipse_paths,n_max=10, name=searching_trajectories[0])
zigzag_aggrigate_time,zigzag_paths_aggrigate_tries = get_aggregates(folder_paths=zigzag_paths,n_max=10, name=searching_trajectories[0])

# Count the occurrences of each value
elipse_value_counts = Counter(elipse_paths_aggrigate_tries)
# Extract values and their corresponding counts
elipse_values, counts = zip(*elipse_value_counts.items())
# Calculate relative frequencies
total_count = sum(counts)
elipse_relative_frequencies = [count / total_count for count in counts]

width = 0.35  # width of the bars
# Count the occurrences of each value
zigzag_value_counts = Counter(zigzag_paths_aggrigate_tries)
# Extract values and their corresponding counts
zigzag_values, counts = zip(*zigzag_value_counts.items())
# Calculate relative frequencies
total_count = sum(counts)
zigzag_relative_frequencies = [count / total_count for count in counts]

# Set up positions for the bars
zigzag_values = [width + value for value in zigzag_values]
# Create a beautiful plot
ax.bar(elipse_values, elipse_relative_frequencies,width=width, color='skyblue',edgecolor='black', alpha=0.7, label=searching_trajectories[0])
ax.bar(zigzag_values , zigzag_relative_frequencies,width=width, color='salmon',edgecolor='black', alpha=0.7, label=searching_trajectories[1])
ax.set_xlabel('Number of Tries')
ax.set_ylabel('Probability')
ax.vlines(np.mean(elipse_values), 0, 1,color='blue',linestyles='--', label='Mean tries elipse trajectory')
ax.vlines(np.mean(zigzag_values), 0, 1,color='orange',linestyles='--', label='Mean tries zigzag trajectory')
ax.set_ylim(0, 0.5)  # Set y-axis limit to be between 0 and 1
ax.grid(axis='y', linestyle='--', alpha=0.7)
ax.legend(loc='upper right')

# # Show the plot
plt.savefig('/home/anish/Documents/Thesis/Plots/grasp_tries_plot.png', bbox_inches='tight', format='png',dpi=600)


fig,ax = plt.subplots(figsize=(10, 6))

# Plotting the distribution using matplotlib
ax.hist(elipse_aggrigate_time, 5, color='skyblue',edgecolor='black', alpha=0.7, label=searching_trajectories[0])
ax.hist(zigzag_aggrigate_time, 5, color='salmon',edgecolor='black', alpha=0.7, label=searching_trajectories[1])
ax.vlines(np.mean(elipse_aggrigate_time), 0, 9,linestyles='--', color='blue', label='Mean time elipse trajectory')
ax.vlines(np.mean(zigzag_aggrigate_time), 0, 9,linestyles='--',color='orange', label='Mean time zigzag trajectory')
ax.set_xlabel('Time Before Landing [s]]')
ax.set_ylabel('Frequency')
ax.grid(axis='y', linestyle='--', alpha=0.7)
ax.legend(loc='upper right')
ax.set_ylim(0,7.5)






plt.savefig('/home/anish/Documents/Thesis/Plots/grasp_time_plot.png', bbox_inches='tight', format='png',dpi=600)



