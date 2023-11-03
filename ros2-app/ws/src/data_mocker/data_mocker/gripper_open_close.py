# Import MPR121 module.
import numpy as np
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8

OPEN = 1
CLOSED = 0

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('gripper_open_close')
        self.publisher_ = self.create_publisher(Int8, '/gripper/in/gripper_state', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gripper_state = 1
        self.start_time = time.perf_counter_ns()

    def timer_callback(self):
        now = time.perf_counter_ns()
        elapsed_time = (now - self.start_time)*1e-9
        self.get_logger().info(f'Elapsed time {elapsed_time}')
        if elapsed_time <= 5:
            msg = Int8()
            msg.data = OPEN
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else: 
            msg = Int8()
            msg.data = CLOSED
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

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