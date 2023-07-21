import time
# Import MPR121 module.
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

class MotorController(Node):

    def __init__(self):
        super().__init__('mpr121_publisher')
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/motor_position_reference', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = UInt16MultiArray()
        msg.data = [1042,1042,1042]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()