from matplotlib import font_manager
from matplotlib import patches
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import matplotlib.cm as ColorMapper
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator
from constants import *
from data_loader import get_data_dict
import numpy as np
import pandas as pd

# Specify the font file path
font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'

# Register the font
font_manager.fontManager.addfont(font_path)

plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'Times New Roman'



state_mapping = {IDLE : 'Idle',
                HOVER: 'Hover',
                MOVING:'Moving',
                TOUCHED: 'Touched',
                GRASP: 'Grasping',
                EVALUATE: 'Evaluating',
                REFINE: 'Refining',
                SEARCHING: 'Searching',
                LAND:'Landing'}

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

# Function to find intervals
def find_intervals(sensor_data):
    intervals = {}
    for column in sensor_data.columns:
        on_intervals = []
        is_on = False
        start_time = None

        for index, value in sensor_data[column].items():
            if value == 1 and not is_on:
                is_on = True
                start_time = index
            elif value == 0 and is_on:
                is_on = False
                on_intervals.append((start_time, index))

        # Check if the last interval continues until the end of the data
        if is_on:
            on_intervals.append((start_time, sensor_data.index[-1]))

        intervals[column] = on_intervals

    return intervals



# File path to rosbag
path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/closed_loop_tactile_7_12/test_tactile_4'
data_dict = get_data_dict(path)

df_ref = data_dict[TRAJECTORY_SETPOINT]
df_mocap = data_dict[MOCAP]
df_est = data_dict[VEHICLE_ODOMETRY]
df_bar = data_dict[BAR_POSE]
df_sensors = data_dict[TACTILE_DATA]
df_sensors = df_sensors.mask(df_sensors - df_sensors.iloc[0] <= -15, 1)
df_sensors = df_sensors.mask(df_sensors > 1, 0)
df_command = data_dict[DRONE_STATE]


# # COMMAND POST-PROCESSOR
# time_of_touch = min(df_sensors[df_sensors.T.sum() >= 1].index)
# time_to_grasp = time_of_touch + 0.5
# df_command['state'].loc[time_of_touch:time_to_grasp] = TOUCHED

# stable_grasp = min(df_sensors[((df_sensors - np.array([1,0,0,1,0,0,1,0,0,0,0,0])).T.sum() >=0)].index)
# end_grasp = stable_grasp - 0.5
# df_command['state'].loc[time_to_grasp:end_grasp] = GRASP
# landing_index = df_sensors.loc[max(df_command.index):max(df_sensors.index)].index
# landing_data = np.array([LAND for i in range(len(landing_index))])
# df_land = pd.DataFrame(data=landing_data,index=landing_index,columns=['state'])
# df_command = df_command._append(df_land)

searching = df_command[df_command['state'] == SEARCHING].index
touched = df_command[df_command['state'] == TOUCHED].index
evaluating = df_command[df_command['state'] == EVALUATE].index
landing = df_command[df_command['state'] == LAND].index

t_start = min(searching)
t_end = min(landing) + 0.5

df_ref = df_ref.loc[t_start:t_end]
df_mocap = df_mocap.loc[t_start:t_end]
df_est = df_est.loc[t_start:t_end]
df_bar = df_bar.loc[t_start:t_end]
df_sensors = df_sensors.loc[t_start:t_end]
df_command = df_command.loc[t_start:t_end]




axes = ['x','y','z']
fig,ax = plt.subplots(4,1, figsize=(25,10))


max_ticks = 4
for i,axes in enumerate(axes):
    ax[i].plot(df_ref.index,df_ref[axes], label='Reference position')
    # ax[i].plot(df_est.index,df_est[axes], label='estimated odometry')
    ax[i].plot(df_mocap.index,df_mocap[axes], label='Vehicle odometry')
    ax[i].hlines(df_bar[axes].iloc[-1],xmin=min(df_bar.index),xmax=max(df_bar.index),linestyles='dashed', label='Bar position')
    ax[i].set_ylabel(f'{axes.capitalize()} [m]')
    ax[i].set_xticks([])
    ax[i].set_xticklabels([])
    ax[i].legend(fontsize=8, loc='upper right')
    ax[i].yaxis.set_major_locator(MaxNLocator(integer=True, prune='both', nbins=max_ticks))


sensor_data = find_intervals(df_sensors)
ax[3].set_yticks([0,1,2,3,4,5,6,7,8])
for i, (sensor, intervals) in enumerate(sensor_data.items()):
    for interval in intervals:
        start_time, end_time = interval
        ax[3].fill_betweenx(y=[i], x1=start_time, x2=end_time,color='blue',linewidth=3, label=sensor)
            

ax[3].set_yticklabels([sensor_name_mapping[i] for i in range(9)])

cmap = ColorMapper.get_cmap('Pastel1')
# Create a color list for each unique state
state_colors = [cmap(i) for i in range(df_command['state'].nunique() + 1)]
custom_cmap = ListedColormap(state_colors)    
for i in range(4):
    # ax[i].tick_params(axis='both', which='major')
    ax[i].imshow([df_command['state'].values], cmap=custom_cmap, aspect='auto', extent=[df_command.index.min(), df_command.index.max(), -10, 10], alpha=0.3)
    
    
    
    
    

        
 # Mischelaneous   
color_mappnig_matrix = {'Searching':custom_cmap(4),
                        'Touched':custom_cmap(0),
                        'Grasping':custom_cmap(1),
                        'Evaluating':custom_cmap(2),
                        'Landing':custom_cmap(5)}
    
legend_handles = [Patch(color=color, label=label) for label,color in color_mappnig_matrix.items()]
fig.legend(handles=legend_handles,loc='upper left', fontsize=8)

ax[3].grid()   
ax[3].set_ylim(0,8)
ax[3].set_xlabel('Time [s]')
ax[0].set_ylim(-0.9,-0.2)
ax[1].set_ylim(-1.5,-0.5)
ax[2].set_ylim(-1.8,-1)
ax[2].invert_yaxis()

# Create rectangles and ellipses for fingers and sensing pads
finger_height = 0.33  # Adjust as needed
finger_width = 0.015
ellipse_radius = 0.015

# Finger representation
ax2 = ax[3].inset_axes([1.01, 0.01, 0.5, 1], transform=ax[3].transAxes)  # Create inset axes to the right

ax2.axis('off')

sensor_id = 0 
gap = 0.07  # Adjust the gap between fingers

for i in range(3):
    # Draw the finger rectangle
    finger_rect = patches.Rectangle((0, i * (finger_height + gap)), finger_width, finger_height, edgecolor='black',
                                   facecolor='none')
    ax2.add_patch(finger_rect)
    # Draw ellipses for sensing pads
    for j in range(3):
        sensing_pad_state = df_sensors[f'sensor_{sensor_id}'].iloc[-1]
        ellipse_color = 'orange' if sensing_pad_state == 1 else 'white'
        y_coordinate =  (i * (finger_height + gap +0.02)) + 0.10 * j
        ellipse = patches.Ellipse((0.007, y_coordinate),
                                  ellipse_radius, ellipse_radius * 2, edgecolor='black', facecolor=ellipse_color)
        ax2.add_patch(ellipse)
        sensor_id += 1
ax2.set_ylim(-0.03, 3 * (finger_height + gap))
    
plt.savefig('/home/anish/Documents/Thesis/Plots/tactile_plot_refine_grasp.png', format='png',dpi=600)
# plt.show()