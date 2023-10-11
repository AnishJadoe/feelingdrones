import time
# Import MPR121 module.
import board
import adafruit_mpr121
import busio
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8MultiArray


NUMBER_OF_TOUCH_PADS = 12
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mpr121_publisher')
        self.publisher_ = self.create_publisher(Int8MultiArray, '/touch_sensor/events', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Create I2C bus.
        self.i2c = busio.I2C(board.SCL, board.SDA)
        # Create MPR121 object.
        self.mpr121 = adafruit_mpr121.MPR121(self.i2c)


    def timer_callback(self):
        msg = Int8MultiArray()
        touch_event = self.check_for_touch_event()
        msg.data = touch_event
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def check_for_touch_event(self):
        touch_events = [0] * NUMBER_OF_TOUCH_PADS
        for i in range(NUMBER_OF_TOUCH_PADS):
            if self.mpr121[i].value:
                touch_events[i] = 1
        return touch_events


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