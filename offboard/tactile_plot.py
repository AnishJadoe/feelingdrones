from matplotlib import font_manager
from matplotlib import patches
from matplotlib.colors import ListedColormap
from matplotlib.offsetbox import AnnotationBbox
import matplotlib.pyplot as plt
import matplotlib.cm as ColorMapper
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator
from constants import *
from data_loader import get_data_dict
from common import getImage
import pandas as pd

# Specify the font file path
font_path = '/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf'

# Register the font
font_manager.fontManager.addfont(font_path)

plt.rcParams['font.size'] = 25
plt.rcParams['font.family'] = 'Times New Roman'
fig_size = (20,10)
dpi = 250



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
        6:'Bot Pad 1', 
        7:'Mid Pad 1',
        8:'Top Pad 1',
        3:'Bot Pad 2',
        4:'Mid Pad 2',
        5:'Top Pad 2',
        0:'Bot Pad 3',
        1:'Mid Pad 3',
        2:'Top Pad 3',
        9:'EMPTY',
        10:'EMPTY',
        11:'EMPTY'
    }

def find_first_touch(sensor_data):
    min_timestamp = float('inf')
    min_sensor_id = None

    for sensor_id, timestamps in sensor_data.items():
        # Check if the sensor has any data
        if timestamps:
            # Get the minimum timestamp for the sensor
            sensor_min_timestamp = min(timestamps, key=lambda x: x[0])[0]

            # Update the minimum timestamp and corresponding sensor id if needed
            if sensor_min_timestamp < min_timestamp:
                min_timestamp = sensor_min_timestamp
                min_sensor_id = sensor_id
    return (min_sensor_id, min_timestamp)
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
fig,ax = plt.subplots(4,1, figsize=fig_size, height_ratios=[1,1,1,2])


max_ticks = 4
for i,axes in enumerate(axes):
    line1 = ax[i].plot(df_ref.index,df_ref[axes], label='Reference position', linewidth=3)
    # ax[i].plot(df_est.index,df_est[axes], label='estimated odometry')
    line2 = ax[i].plot(df_mocap.index,df_mocap[axes], label='Vehicle odometry', linewidth=3)
    line3 = ax[i].hlines(df_bar[axes].iloc[-1],xmin=min(df_bar.index),xmax=max(df_bar.index),linestyles='dashed', label='Bar position', linewidth=3)
    ax[i].set_ylabel(f'{axes.capitalize()} [m]')
    ax[i].set_xticks([])
    ax[i].set_xticklabels([])
    ax[i].yaxis.set_major_locator(MaxNLocator(integer=True, prune='both', nbins=max_ticks))

# Combine the legends from both axes
lines = [line1[0], line2[0], line3]
labels = [line.get_label() for line in lines]


# Create a figure-level legend
fig.legend(lines,labels, loc='upper left',fontsize=20)

sensor_data = find_intervals(df_sensors)
first_touch = find_first_touch(sensor_data)
sensor_data[first_touch[0]][0] = (first_touch[1],first_touch[1]+0.2)

ax[3].set_yticks([0,1,2,3,4,5,6,7,8])
for i, (sensor, intervals) in enumerate(sensor_data.items()):
    for interval in intervals:
        start_time, end_time = interval
        ax[3].fill_betweenx(y=[i], x1=start_time, x2=end_time,color='blue',linewidth=5, label=sensor)
            

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
fig.legend(handles=legend_handles,loc='upper right',fontsize=20)

ax[3].grid()   
ax[3].set_ylim(0,8)
ax[3].set_xlabel('Time [s]')
ax[0].set_ylim(-0.9,-0.2)
ax[1].set_ylim(-1.5,-0.5)
ax[2].set_ylim(-1.8,-1)
ax[2].invert_yaxis()

# Create rectangles and ellipses for fingers and sensing pads

finger_width = 0.015
ellipse_radius = 0.015

# Finger representation
ax2 = ax[3].inset_axes([1.005, -0.05, 0.5, 1.1], transform=ax[3].transAxes)  # Create inset axes to the right

ax2.axis('off')

sensor_id = 0 
finger_id = 0 
gap = 0.07  # Adjust the gap between fingers
transform_ax3_to_ax2 = ax[3].transData + ax2.transData.inverted()
finger_height = transform_ax3_to_ax2.transform((0,2))[1] - transform_ax3_to_ax2.transform((0,0))[1]
# ax2.set_ylim(-0.05, 1.05)
for i in range(3):
    # Draw the finger rectangle

    finger_rect = patches.Rectangle((0, transform_ax3_to_ax2.transform((0,finger_id))[1]), finger_width, finger_height, edgecolor='black',
                                   facecolor='grey', alpha=0.3)
    ax2.add_patch(finger_rect)
    # Draw ellipses for sensing pads
    for j in range(3):
        sensing_pad_state = df_sensors[f'sensor_{sensor_id}'].iloc[-1]
        ellipse_color = 'orange' if sensing_pad_state == 1 else 'white'
        if sensor_id == 8:
            ellipse_color = 'orange'
        y_coordinate =  transform_ax3_to_ax2.transform((0,sensor_id))[1] #(i * (finger_height + gap +0.02)) + 0.14 * j
        ellipse = patches.Ellipse((0.008, y_coordinate),
                                  ellipse_radius, ellipse_radius * 2, edgecolor='black', facecolor=ellipse_color)
        ax2.add_patch(ellipse)
        sensor_id += 1
    finger_id += 3
    

# Identify transitions between states 5 and 8
transitions = df_command['state'].diff()

# Find the indices where transitions occur (from any state to 5 or 8)
transition_indices = transitions[transitions.isin([-1, 1])].index
max_y = [1.7,1.1,1.1,1.1]
min_y = [-0.1,-0.1,-0.1,0]
# Plot vertical lines at transition points
for i in range(4):
    for index in transition_indices:
        ax[i].axvline(x=index, color='black', linestyle='--', label='State Transition',
                    ymin=min_y[i], ymax=max_y[i], clip_on=False)
    
closed_drone_path = '/home/anish/dev/working/feelingdrones/px4_logger/icons/closed_drone.png'
open_drone_path = '/home/anish/dev/working/feelingdrones/px4_logger/icons/open_drone.png'

ax_top = ax[0].inset_axes([0, 1.05, 1, 0.3], transform=ax[0].transAxes)  # Create inset axes to the right
ax_top.axis('off')
transform_ax0_to_ax_top = ax[0].transData + ax_top.transData.inverted()

transition_indices = transition_indices.append(pd.Index([min(landing)]))
transition_indices = transition_indices.insert(0,pd.Index([min(searching)]))
for i in range(len(transition_indices) - 1 ):
    if transition_indices[i+1] - transition_indices[i] < 2:
        continue 
    drone_state = int(df_command.loc[transition_indices[i]:transition_indices[i+1]].iloc[0])
    x = (transition_indices[i+1] + transition_indices[i]) / 2
    if drone_state == GRASP or drone_state == SEARCHING:
        ab = AnnotationBbox(getImage(open_drone_path, zoom=0.12), (transform_ax0_to_ax_top.transform((x, 0.01))),boxcoords="axes fraction", frameon=False)
        ax_top.add_artist(ab)
    if drone_state == EVALUATE:
        ab = AnnotationBbox(getImage(closed_drone_path, zoom=0.12), (transform_ax0_to_ax_top.transform((x, 0.01))),boxcoords="axes fraction", frameon=False)
        ax_top.add_artist(ab)
    
#plt.savefig('/home/anish/Documents/Thesis/Plots/tactile_plot_first_try.png',format='png',dpi=dpi)
plt.show()