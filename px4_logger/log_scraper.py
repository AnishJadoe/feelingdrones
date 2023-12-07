import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon 
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
from pathlib import Path
import json
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

test_name = 'test_tactile_10'
# File path to rosbag
path =f'/home/anish/Documents/Thesis/Drone/ros2_bag_files/offset_0/{test_name}'

# Topics to collect data from
topics=['/rosout']
data = []
##############################################################
############## Load all the data #############################
##############################################################
# create reader instance and open for reading
with Reader(path) as reader:

    # Init the arrays for plotting
    t_ref = []
    reference = []
    reference_yaw = []
    # topic and msgtype information is available on .connections list
    for connection in reader.connections:
        pass
    topics_set = set()
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        topics_set.add(connection.topic)
        if connection.topic == topics[0]:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            timestamp = timestamp * 1e-9
            if timestamp > 20:
                data_entry = {'timestamp':timestamp,
                            'message': msg.msg}
                data.append(data_entry)

with open(f"{test_name}_log.json", "w") as file:
   json.dump(data, file)
