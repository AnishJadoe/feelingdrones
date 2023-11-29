import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
from pathlib import Path

from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd

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
path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/28_11/test_tactile_10'

# Topics to collect data from
topics=['/fmu/in/trajectory_setpoint',
        '/fmu/out/vehicle_odometry',
        '/fmu/in/vehicle_visual_odometry',
        '/drone/out/state',
        '/bar/pose',]

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
bar = np.array([[p.x,p.y,p.z] for p in bar])
bar_q = np.array([[q.x, q.y, q.z, q.w] for q in bar_q])

t_command = np.array(t_command, dtype=float)
command = np.array(command,dtype=int)
# Normalize time 
start_t = min(np.concatenate((t_ref, t_odom, t_mocap, t_bar, t_sensors, t_command)))
t_ref = (t_ref - start_t) * 1e-9
t_odom = (t_odom - start_t) * 1e-9
t_mocap = (t_mocap - start_t) * 1e-9
t_bar = (t_bar - start_t) * 1e-9
t_sensors = (t_sensors - start_t) * 1e-9
t_command = (t_command - start_t) * 1e-9
cmap = 'viridis'
norm = Normalize(vmin=min(t_ref), vmax=max(t_ref))
sm = ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])

# x = reference[:,0]
# y = reference[:,1]
# fig,ax = plt.subplots()
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# scatter = ax.scatter(x, y, c=t_ref, cmap=cmap, norm=norm, marker='o', s=10, alpha=1)
# cbar = fig.colorbar(sm, ax=ax, label='Time')
# ax.annotate('Beginning', (x[0],y[0]))
# plt.show()
xmax = 1
xmin = -1
ymax = 1
ymin = -1

bar_x = bar[:,1]
bar_y = bar[:,0]
bar_z = -bar[:,2]
df_ref = pd.DataFrame(reference,columns=['x','y','z'], index=t_ref)
df_est = pd.DataFrame(odom,columns=['x','y','z'], index=t_odom)
df_command = pd.DataFrame(command, columns=['state'], index=t_command)
x_ref = df_ref[df_ref['x'] > 0]['x']
y_ref = df_ref[df_ref['x'] > 0]['y']
z_ref = df_ref[df_ref['x'] > 0]['z']

time_searching = df_command[(df_command['state'] == 7)].index
time_grasping = df_command[(df_command['state'] == 4)].index
time_evaluating = df_command[(df_command['state'] == 5)].index

t_start = min(df_ref.index)
t_end = 80 #max(df_ref.index)
df_est = df_est.loc[t_start:t_end]
df_ref = df_ref.loc[t_start:t_end]
x_est = df_est['x']
y_est = df_est['y']
z_est = df_est['z']

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.scatter(x_ref, y_ref, z_ref,marker='o', s=10, alpha=0.2, label='Drone Reference Trajectory')
ax.scatter(x_est, y_est, z_est,c=x_est.index,cmap=cmap, norm=norm,label='Drone Estimated Position', marker='o', s=10, alpha=0.9)
# ax.scatter(x_est.iloc[0], y_est[0], z_est[0], color='red', label='Touch Location')

cbar = fig.colorbar(sm, ax=ax, label='Time')
# ax.annotate('Beginning', (df_ref['x'].iloc[0],df_ref['y'].iloc[0]))

# Plot the axis line representing the object
object_center = (bar_x[0], bar_y[0], bar_z[0])
axis_length = 1  # Adjust the axis length as needed
ax.plot([object_center[0] - axis_length / 2, object_center[0] + axis_length / 2], [object_center[1], object_center[1]], [object_center[2], object_center[2]], color='blue', label='Object Axis', linewidth=5)
ax.legend()
ax.invert_zaxis()
plt.show()