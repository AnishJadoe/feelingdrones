from dataclasses import dataclass
from pathlib import Path
import pandas as pd
import numpy as np
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

def init_reader():
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

def get_position_data(topic, reader):
    # Init the arrays for plotting
    data = []
    t_ref = []
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        pass
    topics_set = set()
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        topics_set.add(connection.topic)
        if connection.topic == topic:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_ref += [timestamp]
            data += [msg.position] 
            
    df = pd.DataFrame(data=data, columns=['x', 'y','z'], index=t_ref)
    return df

def get_sensor_data(topic,reader):
    # Init the arrays for plotting
    sensor_data = [[],[],[],[],[],[],[],[],[],[],[],[]] 
    t_ref = []
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        pass
    topics_set = set()
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        topics_set.add(connection.topic)
        if connection.topic == topic:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t_ref += [timestamp]
            for idx,data in enumerate(msg.data):
                sensor_data[idx].append(data)
    sensor_data = np.array(sensor_data)
    df = pd.DataFrame(data=sensor_data.T, columns=[f'sensor_{i}' for i in range(12)], index=t_ref)
    return df

def get_command_data(topic,reader):

    # Init the arrays for plotting
    data = []
    t_ref = []
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        pass
    topics_set = set()
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        topics_set.add(connection.topic)
        if connection.topic == topic:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            data += [msg.data] 
            t_ref += [timestamp]
    df = pd.DataFrame(data, columns=['state'], index=t_ref)
    return df

def get_bar_data(topic, reader):
    # Init the arrays for plotting
    data = []
    t_ref = []
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        pass
    topics_set = set()
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        topics_set.add(connection.topic)
        if connection.topic == topic:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            data += [msg.pose.position] 
            t_ref += [timestamp]
    df = pd.DataFrame(data=data, columns=['x', 'y','z'], index=t_ref)
    return df

def normalize_time(data_dict):
    data_entries = data_dict.keys()
    start_t = min(np.concatenate([data_dict[key].index for key in data_entries]))
    
    for key in data_entries:
        data_dict[key].index = (data_dict[key].index - start_t ) * 1e-9
    return data_dict
    
MAPPING_NAMING = {
    '/fmu/in/vehicle_visual_odometry': 'mocap',
    '/fmu/out/vehicle_odometry':'vehicle_odometry',
    '/fmu/in/trajectory_setpoint': 'trajectory_setpoint',
    '/touch_sensor/raw_data' : 'tactile_data',
    '/drone/out/state': 'drone_state',
    '/bar/pose':'bar_pose'
}

MAPPING_PARSER = {
    '/fmu/in/vehicle_visual_odometry': get_position_data,
    '/fmu/out/vehicle_odometry':get_position_data,
    '/fmu/in/trajectory_setpoint': get_position_data,
    '/bar/pose': get_bar_data,
    '/touch_sensor/raw_data' :get_sensor_data,
    '/drone/out/state': get_command_data,
}




# Topics to collect data from
topics=['/fmu/in/trajectory_setpoint',
        '/fmu/out/vehicle_odometry',
        '/fmu/in/vehicle_visual_odometry',
        '/drone/out/state',
        '/bar/pose',
        '/touch_sensor/raw_data']


def get_data_dict(file_path):
    data_dict = {}
    init_reader()
    # create reader instance and open for reading
    with Reader(file_path) as reader:
        for topic in topics:
            name = MAPPING_NAMING[topic]
            parser = MAPPING_PARSER[topic]
            data_dict[name] = parser(topic,reader)
    data_dict = normalize_time(data_dict)
    return data_dict


