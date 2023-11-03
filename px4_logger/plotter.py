import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
from pathlib import Path

from rosbags.typesys import get_types_from_msg, register_types
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

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
    '/home/anish/dev/working/feelingdrones/ros2-app/ws/src/custom_msgs/msg/StampedInt32MultiArray.msg'
]:
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))

register_types(add_types)


# File path to rosbag
path ='/home/anish/Documents/Thesis/Drone/ros2_bag_files/3_11/test_tactile_7/'

# Topics to collect data from
topics=['/fmu/in/trajectory_setpoint',
        '/fmu/out/vehicle_odometry',
        '/fmu/in/vehicle_visual_odometry',
        '/fmu/in/vehicle_command',
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
            command += [msg.confirmation]
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
bar = np.array([[p.x,p.y,p.z] for p in bar])
bar_q = np.array([[q.x, q.y, q.z, q.w] for q in bar_q])


base_q0 = odom_q[:,0]
base_q1 = odom_q[:,1]
base_q2 = odom_q[:,2]
base_q3 = odom_q[:,3]
base_yaw = np.arctan2(
        2 * ((base_q1 * base_q2) + (base_q0 * base_q3)),
        base_q0**2 + base_q1**2 - base_q2**2 - base_q3**2
    ) - np.pi/2


# mocap_q0 = mocap_q[:,0]
# mocap_q1 = mocap_q[:,1]
# mocap_q2 = mocap_q[:,2]
# mocap_q3 = mocap_q[:,3]
# mocap_yaw = np.arctan2(
#         2 * ((mocap_q1 * mocap_q2) + (mocap_q0 * mocap_q3)),
#         mocap_q0**2 + mocap_q1**2 - mocap_q2**2 - mocap_q3**2
#     ) - np.pi/2

# Normalize time 
start_t = min(np.concatenate((t_ref, t_odom, t_mocap, t_bar, t_sensors)))
t_ref = (t_ref - start_t) * 1e-9
t_odom = (t_odom - start_t) * 1e-9
t_mocap = (t_mocap - start_t) * 1e-9
t_bar = (t_bar - start_t) * 1e-9
t_sensors = (t_sensors - start_t) * 1e-9




plt.show()
fig,ax = plt.subplots(2,2)
ax[0][0].plot(t_ref,reference[:,0], label='reference')
ax[0][0].plot(t_odom,odom[:,0], label='odometry')
ax[0][0].plot(t_mocap,mocap[:,0], label='mocap')
# ax[0][0].plot(t_bar,bar[:,0], label='bar')
ax[0][0].set_title('X')
ax[0][0].legend()

ax[0][1].plot(t_ref,reference[:,1], label='reference')
ax[0][1].plot(t_odom,odom[:,1], label='odometry')
ax[0][1].plot(t_mocap,mocap[:,1], label='mocap')
# ax[0][1].plot(t_bar,bar[:,1], label='bar')
ax[0][1].set_title('Y')
ax[0][1].legend()

ax[1][0].plot(t_ref,reference[:,2], label='reference')
ax[1][0].plot(t_odom,odom[:,2], label='odometry')
ax[1][0].plot(t_mocap,mocap[:,2], label='mocap')
# ax[1][0].plot(t_bar,-1*bar[:,2], label='bar')
ax[1][0].set_title('Z')
ax[1][0].legend()
for i in range(len(tactile_sensors)):
        ax[1][0].plot(t_sensors,tactile_sensors[i],label=f'sensor_{i}')
plt.show()

# ax[1][1].plot(t_ref,reference_yaw, label='reference')
# ax[1][1].plot(t_odom,np.rad2deg(base_yaw), label='odometry')
# ax[1][1].plot(t_mocap,np.rad2deg(mocap_yaw), label='mocap')
# ax[1][1].set_title('Yaw')
# ax[1][1].legend()



