# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import sys

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.transform_broadcaster import TransformBroadcaster


# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.




def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

NED_ENU_Q = quaternion_from_euler(np.pi, 0.0, 2*np.pi).reshape(4,1)

def enu_2_ned(enu):
        ned = np.array([enu[1], enu[0], -enu[2]]).reshape(3,1)
        return ned

def ned_2_enu(ned):
        enu = np.array([ned[1], ned[0], -ned[2]]).reshape(3,1)
        return enu

def q_enu_2_ned(enu):
    ned = NED_ENU_Q*enu
    return ned

def q_ned_2_enu(ned):
    enu = NED_ENU_Q*ned
    return enu

class FramePublisher(Node):
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('tf2_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
            PoseStamped,
            f'/optitrack_pose',
            self.handle_optitrack_pose,
            1)
        self.subscription  # prevent unused variable warning


    def handle_optitrack_pose(self, msg:PoseStamped):
        logger = rclpy.logging.get_logger('logger')
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'NED'
        # ENU to NED transformation
        enu_p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]).reshape(3,1)
        ned_p = enu_2_ned(enu_p)
        t.transform.translation.x = 0.0#float(ned_p[0])
        t.transform.translation.y = 0.0 #float(ned_p[1])
        t.transform.translation.z = 0.0 #float(ned_p[2])

        # NED to ENU orientation transformation
        enu_q = np.array([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z, msg.pose.orientation.w]).reshape(4,1)
        ned_q = q_enu_2_ned(enu_q)
        t.transform.rotation.x = float(ned_q[0])
        t.transform.rotation.y = float(ned_q[1])
        t.transform.rotation.z = float(ned_q[2])
        t.transform.rotation.w = float(ned_q[3])

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)



def main():
    rclpy.init()
    print('---------------------------------------------------------------------')
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
