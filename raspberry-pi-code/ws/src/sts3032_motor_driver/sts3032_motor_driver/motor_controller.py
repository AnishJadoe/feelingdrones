import time
# Import MPR121 module.
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.publisher_ = self.create_publisher(UInt16MultiArray, '/motor_position_reference', 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('init_count', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('max_count', rclpy.Parameter.Type.INTEGER)
        self.counter = self.get_parameter('init_count').value
        self.max_count = self.get_parameter('max_count').value

    

    def timer_callback(self):
        if self.counter == self.max_count:
            self.counter = self.get_parameter('init_count').value
        msg = UInt16MultiArray()
        position = 1042 * self.counter
        msg.data = [position,position,position]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.counter += 1


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