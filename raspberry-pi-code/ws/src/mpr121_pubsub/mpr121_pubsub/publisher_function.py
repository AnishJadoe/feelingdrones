import time
# Import MPR121 module.
import board
import adafruit_mpr121
import busio
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int32MultiArray
from mpr121_pubsub.msg import StampedInt32MultiArray
from mpr121_pubsub.msg import StampedInt8MultiArray
from px4_msgs.msg import TimesyncStatus



NUMBER_OF_TOUCH_PADS = 12
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mpr121_publisher')
        self._touch_publiser = self.create_publisher(Int8MultiArray, '/touch_sensor/events', 10)
        self._raw_data_publisher = self.create_publisher(Int32MultiArray, '/touch_sensor/raw_data', 10)
        self._filtered_data_publisher = self.create_publisher(Int32MultiArray, '/touch_sensor/filtered_data', 10)

        self._timesync_subscriber = self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status", self._time_sync_callback,10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Create I2C bus.
        self.i2c = busio.I2C(board.SCL, board.SDA)
        # Create MPR121 object.
        self.mpr121 = adafruit_mpr121.MPR121(self.i2c)

        self._local_time = None
        self._remote_time = None

    def _time_sync_callback(self, msg):

        self._local_time = time.monotonic()
        self._remote_time = msg.timestamp
        return 
    
    def _get_timestamp(self):
        now = time.monotonic()
        return self._remote_time + (now - self._local_time)
    
    def timer_callback(self):
        touch_event = self.check_for_touch_event()
        raw_data = self.check_for_raw_data()
        filtered_data = self.check_for_filtered_data()

        touch_msg = StampedInt8MultiArray()
        touch_msg.data = touch_event
        touch_msg.timestamp = self._get_timestamp()
        self._touch_publiser.publish(touch_msg)
        
        raw_data_msg = StampedInt32MultiArray()
        raw_data_msg.data = raw_data
        raw_data_msg.timestamp = self._get_timestamp()
        self._raw_data_publisher.publish(raw_data_msg)
     

        filtered_data_msg = StampedInt32MultiArray()
        filtered_data_msg.data = filtered_data
        filtered_data.msg = self._get_timestamp()
        self._filtered_data_publisher.publish(filtered_data_msg)




    def check_for_touch_event(self):
        touch_events = [0] * NUMBER_OF_TOUCH_PADS
        for i in range(NUMBER_OF_TOUCH_PADS):
            if self.mpr121[i].value:
                touch_events[i] = 1
        return touch_events
    
    def check_for_raw_data(self):
        raw_data = [0] * NUMBER_OF_TOUCH_PADS
        for i in range(NUMBER_OF_TOUCH_PADS):
            if self.mpr121[i].value:
                raw_data[i] = self.mpr121.baseline_data(i)
        return raw_data
    
    def check_for_filtered_data(self):
        filtered_data = [0] * NUMBER_OF_TOUCH_PADS
        for i in range(NUMBER_OF_TOUCH_PADS):
                filtered_data[i] = self.mpr121.filtered_data(i)
        return filtered_data


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