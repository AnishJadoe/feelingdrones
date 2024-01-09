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

# Function for guessing ros message tupe
def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)


# Install non-standard types
add_types = {}

for pathstr in [
    '/home/anish/dev/working/feelingdrones/ros2-app/ws/src/px4_msgs/msg/VehicleOdometry.msg',
    '/home/anish/dev/working/feelingdrones/ros2-app/ws/src/px4_msgs/msg/TrajectorySetpoint.msg',
    '/home/anish/dev/working/feelingdrones/ros2-app/ws/src/px4_msgs/msg/VehicleCommand.msg',
    '/home/anish/dev/working/feelingdrones/ros2-app/ws/src/custom_msgs/msg/StampedInt32MultiArray.msg',
    '/home/anish/dev/working/feelingdrones/ros2-app/ws/src/custom_msgs/msg/StampedInt8.msg',
]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)


# File path to rosbag
path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/13_11/test_tactile_20'

# Topics to collect data from
topics=['/fmu/in/trajectory_setpoint',
        '/fmu/out/vehicle_odometry',
        '/fmu/in/vehicle_visual_odometry',
        '/drone/out/state',
        '/bar/pose',
        '/touch_sensor/events']

##############################################################
############## Load all the data #############################
##############################################################

# create reader instance and open for reading
with Reader(path) as reader:

    # Init the arrays for plotting
    t_ref = []
    reference = []
    reference_yaw = []

    t_odom = []
    odom = []
    odom_q = []

    t_mocap = []
    mocap = []
    mocap_q = []

    t_bar = []
    bar = []
    bar_q = []

    t_command = []
    command = []

    t_sensors = []
    tactile_sensors = [[],[],[],[],[],[],[],[],[],[],[],[]] 


    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        pass
    topics_set = set()
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        topics_set.add(connection.topic)
        if connection.topic == topics[0]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_ref += [timestamp]
            reference += [msg.position] 
            reference_yaw += [msg.yaw]
            
        if connection.topic == topics[1]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_odom += [timestamp]
            odom += [msg.position]
            odom_q += [msg.q]

        if connection.topic == topics[2]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_mocap += [timestamp]
            mocap += [msg.position]
            mocap_q += [msg.q]
        if connection.topic == topics[3]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_command += [timestamp]
            command += [msg.data]
        if connection.topic == topics[4]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_bar += [timestamp]
            bar += [msg.pose.position]
            bar_q += [msg.pose.orientation]
        if connection.topic == topics[5]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_sensors += [timestamp]
            for idx,data in enumerate(msg.data):
                tactile_sensors[idx].append(data)

# Make all the arrays np arrays
t_ref = np.array(t_ref, dtype=float) 
reference = np.array(reference, dtype=float)
reference_yaw = np.array(reference_yaw, dtype=float)

t_odom = np.array(t_odom, dtype=float) 
odom = np.array(odom, dtype=float)
odom_q = np.array(odom_q, dtype=float)

t_mocap = np.array(t_mocap, dtype=float) 
mocap = np.array(mocap, dtype=float)
mocap_q = np.array(mocap_q, dtype=float)

t_bar = np.array(t_bar, dtype=float) 
bar = np.array([[p.y,p.x,-p.z] for p in bar])
bar_q = np.array([[q.x, q.y, q.z, q.w] for q in bar_q])

t_sensors = np.array(t_sensors,dtype=float)
tactile_sensors = np.array(tactile_sensors)

t_command = np.array(t_command, dtype=float)
command = np.array(command,dtype=int)
# Normalize time 
start_t = min(np.concatenate((t_ref, t_odom, t_mocap, t_bar, t_sensors)))
t_ref = (t_ref - start_t) * 1e-9
t_odom = (t_odom - start_t) * 1e-9
t_mocap = (t_mocap - start_t) * 1e-9
t_bar = (t_bar - start_t) * 1e-9
t_sensors = (t_sensors - start_t) * 1e-9
t_command = (t_command - start_t) * 1e-9

df_ref = pd.DataFrame(data=reference, columns=['x', 'y','z'], index=t_ref)
df_mocap = pd.DataFrame(data=mocap, columns=['x','y','z'], index=t_mocap)
df_est = pd.DataFrame(data=odom, columns=['x', 'y','z'], index=t_odom)
df_bar = pd.DataFrame(data=bar, columns=['x', 'y','z'], index=t_bar)
df_sensors = pd.DataFrame(data=tactile_sensors.T, columns=[f'sensor_{i}' for i in range(12)], index=t_sensors)
df_command = pd.DataFrame(command, columns=['state'], index=t_command)

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

for i in range(len(tactile_sensors)):
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
