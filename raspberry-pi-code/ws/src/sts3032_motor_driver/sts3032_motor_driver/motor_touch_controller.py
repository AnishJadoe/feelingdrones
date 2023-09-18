import time
# Import MPR121 module.
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Int8MultiArray

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.motor_controller_publisher_ = self.create_publisher(UInt16MultiArray, '/motor/in/position_reference', 10)
        self.touch_sensor_subscriber_ = self.create_subscription(
            Int8MultiArray,
            'touch/out/events',
            self.listener_callback,
            10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter('init_count', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('max_count', rclpy.Parameter.Type.INTEGER)
        self.counter = self.get_parameter('init_count').value
        self.max_count = self.get_parameter('max_count').value

    
    def listener_callback(self, msg):

        return
    def timer_callback(self):
        if self.counter == self.max_count:
            self.counter = self.get_parameter('init_count').value


        position = 1042 * self.counter



        msg = UInt16MultiArray()
        msg.data = [position,position,position]
        self.motor_controller_publisher_.publish(msg)
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