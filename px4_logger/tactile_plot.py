from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import matplotlib.cm as ColorMapper
from matplotlib.patches import Patch
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

state_mapping = {IDLE : 'Idle',
                HOVER: 'Hover',
                MOVING:'Moving',
                TOUCHED: 'Touched',
                GRASP: 'Grasping',
                EVALUATE: 'Evaluating',
                REFINE: 'Refining',
                SEARCHING: 'Searching'}

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
path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/28_11/test_tactile_10'
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

t_start = 0
t_end = 120

df_ref = df_ref.loc[t_start:t_end]
df_mocap = df_mocap.loc[t_start:t_end]
df_est = df_est.loc[t_start:t_end]
df_bar = df_bar.loc[t_start:t_end]
df_sensors = df_sensors.loc[t_start:t_end]
df_command = df_command.loc[t_start:t_end]



axes = ['x','y','z']
fig,ax = plt.subplots(4,1)

cmap = ColorMapper.get_cmap('Pastel2', df_command['state'].nunique())
# Create a color list for each unique state
state_colors = [cmap(i) for i in range(df_command['state'].nunique())]
custom_cmap = ListedColormap(state_colors)
for i,axes in enumerate(axes):
    ax[i].plot(df_ref.index,df_ref[axes], label='reference')
    ax[i].plot(df_est.index,df_est[axes], label='estimated odometry')
    ax[i].plot(df_mocap.index,df_mocap[axes], label='mocap')
    ax[i].hlines(df_bar[axes].iloc[-1],xmin=min(df_bar.index),xmax=max(df_bar.index),linestyles='dashed', label='bar')
    ax[i].imshow([df_command['state']], cmap=custom_cmap, aspect='auto', extent=[df_command.index.min(), df_command.index.max(), -10, 10], alpha=0.5)
    ax[i].set_ylabel(f'{axes.capitalize()} [m]', fontsize=5)
    ax[i].set_xticks([])
    ax[i].set_xticklabels([])
    ax[i].legend(fontsize=5, loc='upper right')

# Doing it by hand because i have no idea how to fix this 

color_mappnig_matrix = {'Searching':cmap(3),
                        'Touched':cmap(0),
                        'Grasping':cmap(1),
                        'Evaluating':cmap(2)}
legend_handles = [Patch(color=color, label=label) for label,color in color_mappnig_matrix.items()]

df_sensors = df_sensors.mask(df_sensors - df_sensors.iloc[0] <= -10, 1)
df_sensors = df_sensors.mask(df_sensors > 1, 0)

ax[3].imshow([df_command['state']], cmap=cmap, aspect='auto', extent=[df_command.index.min(), df_command.index.max(), 0, 9], alpha=0.5)
sensor_data = find_intervals(df_sensors)
cmap = ColorMapper.get_cmap('tab10')
color_mapper = {}
for (sensor, intervals) ,color in zip(sensor_data.items(),cmap.colors):
    for interval in intervals:
        start_time, end_time = interval
        ax[3].fill_betweenx(y=[sensor], x1=start_time, x2=end_time,color='blue',linewidth=3, label=sensor)
        color_mapper[sensor] = color

ax[3].set_yticks([0,1,2,3,4,5,6,7,8])
# ax[3].set_yticklabels(list(sensor_data.keys())[:9][-1])
ax[3].set_yticklabels([0,1,2,3,4,5,6,7,8])
ax[3].grid()       
ax[3].set_ylabel('Sensor ID',fontsize=5)
for i in range(4):
    ax[i].tick_params(axis='both', which='major', labelsize=5)
    
fig.legend(handles=legend_handles,loc='upper left', fontsize=5)
fig.suptitle('Drone trajectory and tactile output during a single experiment', fontsize=10)
ax[3].set_xlabel('Time [s]')
ax[0].set_ylim(-5,5)
ax[1].set_ylim(-5,5)
ax[2].set_ylim(-6,-0)
ax[2].invert_yaxis()


plt.show()
# plt.savefig('/home/anish/Documents/Thesis/Plots/tactile_plot_with_evaluation.png', format='png',dpi=1200)
