import time
# Import MPR121 module.
import board
import adafruit_mpr121
import busio
import rclpy
from rclpy.node import Node
from rclpy import qos
from px4_msgs.msg import TimesyncStatus
from custom_msgs.msg import StampedInt32MultiArray
from custom_msgs.msg import StampedInt8MultiArray


MINIMUM_CAPACITANCE_DIFFERENCE = -5
NUMBER_OF_TOUCH_PADS = 12
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mpr121_publisher')
        print("INIT TACTILE SENSOR NODE")
        # self._touch_publiser = self.create_publisher(StampedInt32MultiArray, '/touch_sensor/events',qos.QoSPresetProfiles.SENSOR_DATA.value)
        self._raw_data_publisher = self.create_publisher(StampedInt32MultiArray, '/touch_sensor/raw_data', qos.QoSPresetProfiles.SENSOR_DATA.value)
        self._filtered_data_publisher = self.create_publisher(StampedInt32MultiArray, '/touch_sensor/filtered_data', qos.QoSPresetProfiles.SENSOR_DATA.value)
        self._baseline_data_publisher = self.create_publisher(StampedInt32MultiArray, '/touch_sensor/baseline_data', qos.QoSPresetProfiles.SENSOR_DATA.value)

        self._timesync_subscriber = self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status", self._time_sync_callback,qos.QoSPresetProfiles.SENSOR_DATA.value)
        timer_period = 1e-3  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Create I2C bus.
        self.i2c = busio.I2C(board.SCL, board.SDA)
        # Create MPR121 object.
        self.mpr121 = adafruit_mpr121.MPR121(self.i2c)
        self.sensor_channels = [adafruit_mpr121.MPR121_Channel(self.mpr121, i) for i in range(NUMBER_OF_TOUCH_PADS)]
        self._local_time = time.monotonic_ns()
        self._remote_time = time.monotonic_ns()
        self._init_state = self.check_for_raw_data()

    def _time_sync_callback(self, msg):
        self._local_time = time.monotonic_ns()
        self._remote_time = msg.timestamp
        return 
    
    def _get_timestamp(self):
        now = time.monotonic_ns()
        return self._remote_time + round((now - self._local_time))
    
    def timer_callback(self):

        # touch_event = self.check_for_touch_event()
        # touch_msg = StampedInt32MultiArray()
        # touch_msg.data = touch_event
        # touch_msg.timestamp = self._get_timestamp()
        # self._touch_publiser.publish(touch_msg)
        
        raw_data = self.check_for_raw_data()
        raw_data_msg = StampedInt32MultiArray()
        raw_data_msg.data = raw_data
        raw_data_msg.timestamp = self._get_timestamp()
        self._raw_data_publisher.publish(raw_data_msg)

        baseline_data = self.check_for_baseline_data()
        baseline_data_msg = StampedInt32MultiArray()
        baseline_data_msg.data = baseline_data
        baseline_data_msg.timestamp = self._get_timestamp()
        self._baseline_data_publisher.publish(baseline_data_msg)
     
        filtered_data = self.check_for_filtered_data()
        filtered_data_msg = StampedInt32MultiArray()
        filtered_data_msg.data = filtered_data
        filtered_data_msg.timestamp = self._get_timestamp()
        self._filtered_data_publisher.publish(filtered_data_msg)

    # def check_for_touch_event(self):
    #     touch_events = [0] * NUMBER_OF_TOUCH_PADS
    #     current_tactile_state = [self.sensor_channels[i].raw_value for i in range(NUMBER_OF_TOUCH_PADS)]
    #     cap_difference = [self._init_state[i] - current_tactile_state[i] for i in range(NUMBER_OF_TOUCH_PADS)]
    #     for i in range(len(cap_difference)):
    #         if cap_difference[i] <= MINIMUM_CAPACITANCE_DIFFERENCE:
    #             touch_events[i] = 1
    #         else:
    #             touch_events[i] = 0
    #     return touch_events
    
    def check_for_baseline_data(self):
        return [self.mpr121.baseline_data(i) for i in range(NUMBER_OF_TOUCH_PADS)]
    
    def check_for_raw_data(self):
        return [self.sensor_channels[i].raw_value for i in range(NUMBER_OF_TOUCH_PADS)]
    
    def check_for_filtered_data(self):
        return [self.mpr121.filtered_data(i) for i in range(NUMBER_OF_TOUCH_PADS)]


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
