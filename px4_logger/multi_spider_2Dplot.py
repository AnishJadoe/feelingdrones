from matplotlib import font_manager
from matplotlib.markers import MarkerStyle
from matplotlib.offsetbox import AnnotationBbox, OffsetImage
from matplotlib.patches import Patch
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd
from common import getImage
from constants import *
from data_loader import get_data_dict


# Specify the font file path
font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'

# Register the font
font_manager.fontManager.addfont(font_path)

plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'Times New Roman'




def plot_2D_endpoints(ax, df, translation):
    # Load your SVG file
    closed_drone_path = '/home/anish/dev/working/feelingdrones/px4_logger/icons/closed_drone.png'
    open_drone_path = '/home/anish/dev/working/feelingdrones/px4_logger/icons/open_drone.png'
    
    x = df['x'] + translation[0]
    y = df['y'] + translation[1]
    ax.plot([x.iloc[0],x.iloc[-1]], [y.iloc[0],y.iloc[-1]], '--',linewidth=0.3, color='tab:purple')
    ab = AnnotationBbox(getImage(open_drone_path), (x.iloc[0], y.iloc[0]), frameon=False)
    ax.add_artist(ab)
    ab = AnnotationBbox(getImage(closed_drone_path), (x.iloc[-1], y.iloc[-1]), frameon=False)
    ax.add_artist(ab)
    return ax

def plot_2D_trajectory(ax, df, translation):
    closed_drone_path = '/home/anish/dev/working/feelingdrones/px4_logger/icons/closed_drone.png'
    open_drone_path = '/home/anish/dev/working/feelingdrones/px4_logger/icons/open_drone.png'
    x = df['x'] + translation[0]
    y = df['y'] + translation[1]

    y_start = df['y'].head(1)
    y_end = df['y'].tail(1)
    
    y = pd.DataFrame(y[abs(y) < 0.14].sample(100).sort_index())
    y.loc[y_start.index[0]] = y_start
    y.loc[y_end.index[0]] = y_end
 
    x = x.loc[y.index]
    ax.plot(x,y,'--',linewidth=0.1)
    ab = AnnotationBbox(getImage(open_drone_path), (x.iloc[0], y.iloc[0]), frameon=False)
    ax.add_artist(ab)
    ab = AnnotationBbox(getImage(closed_drone_path), (x.iloc[-1], y.iloc[-1]), frameon=False)
    ax.add_artist(ab)
    return ax

# File path to rosbag
HOME = '/home/anish/Documents/Thesis/Drone/ros2_bag_files/'
folder_paths = ['offset_0','offset_05', 'spider_plot', '28_11','27_11', 'closed_loop_tactile_6_12', 'closed_loop_tactile_7_12', 'zigzag_7_12']


fig,ax = plt.subplots(figsize=(10,6))
bar_x = 0
bar_y = 0
object_center = (bar_x,bar_y)
axis_length = 1  # Adjust the axis length as needed
ax.plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], color='grey', linewidth=5, label='Perching Object')
n_plots = 0
for folder_path in folder_paths:
    for path in os.listdir(HOME+folder_path):
        data_dict = get_data_dict(f'{HOME+folder_path}/{path}')
        df_mocap = data_dict[MOCAP]
        df_bar = data_dict[BAR_POSE]
        df_command = data_dict[DRONE_STATE]
        df_sensors = data_dict[TACTILE_DATA]

        time_searching = df_command[(df_command['state'] == SEARCHING)].index
        time_grasping = df_command[(df_command['state'] == GRASP)].index
        time_evaluating = df_command[(df_command['state'] == EVALUATE)].index
        time_touched = df_command[(df_command['state'] == TOUCHED)].index

        if time_evaluating.empty or df_sensors.empty or time_searching.empty:
            continue
        
        try:
            df_sensors = df_sensors.mask(df_sensors - df_sensors.iloc[0] <= -15, 1)
            df_sensors = df_sensors.mask(df_sensors > 1, 0)
            landing_tactile_state = np.array(df_sensors.loc[max(time_evaluating):].iloc[0])
        except:
            continue
        
        bar_x = df_bar['x'].iloc[-1]
        bar_y = df_bar['y'].iloc[-1]
        translation = np.array([[object_center[0] - bar_x],[object_center[1] - bar_y]])
        stable_grasp = (
            (sum(landing_tactile_state - np.array([0,1,0,0,1,0,0,1,0,0,0,0])) >= 0) \
            or (sum(landing_tactile_state - np.array([1,0,0,1,0,0,1,0,0,0,0,0])) >= 0 )
            ) \
            and not (sum(landing_tactile_state) > 9) # every sensor on
        
        t_start = max(time_searching)
        t_end = max(time_evaluating)
        df_mocap = df_mocap.loc[t_start:t_end]
        if stable_grasp:
            print(f"Plotting {path} ")
            n_plots += 1
            plot_2D_endpoints(ax,df_mocap, translation)
        

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_ylim(-0.15,0.15)
ax.set_xlim(-0.2,0.45)

ax.hlines(0.14,-1,1,label='Length top phalange', color='red', linewidth=0.3)
ax.hlines(-0.14,-1,1, color='red', linewidth=0.3)

ax.hlines(0.09,-1,1,label='Length mid phalange', colors='blue', linewidth=0.3)
ax.hlines(-0.09,-1,1, colors='blue', linewidth=0.3)

ax.hlines(0.04,-1,1,label='Length bottom phalange', colors='green', linewidth=0.3)
ax.hlines(-0.04,-1,1, colors='green', linewidth=0.3)

# color_mappnig_matrix = {'Grasp Position': 'green',
#                         'Touch Position':'red'}
# legend_handles = [Patch(color=color, label=label) for label,color in color_mappnig_matrix.items()]

# Plot the axis line representing the object
ax.legend(loc='lower right')
#plt.show()
print(f"Made spider plot for {n_plots} experiments")
plt.savefig('/home/anish/Documents/Thesis/Plots/spider_plot.png',bbox_inches='tight', format='png',dpi=600)
