# Import MPR121 module.
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8

OPEN = 1
CLOSED = 0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mpr121_publisher')
        self.publisher_ = self.create_publisher(Int8, '/gripper/out/gripper_state', 10)
        self.subscriber = self.create_subscription(Int8,'/gripper/in/gripper_state', self.gripper_callback, 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gripper_state = 1

    def timer_callback(self):
        msg = Int8()
        msg.data = self.gripper_state
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def gripper_callback(self, msg):
        if msg.data == OPEN:
            self.gripper_state = OPEN
        if msg.data == CLOSED:
            self.gripper_state = CLOSED 
        return



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()