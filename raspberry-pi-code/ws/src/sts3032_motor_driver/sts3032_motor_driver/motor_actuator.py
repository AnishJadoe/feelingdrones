from dataclasses import dataclass, fields,asdict
import dataclasses 
from typing import List, Optional, Tuple, Union
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import serial
import struct
from time import sleep
from math import floor


MAX_SERVO_COMMAND = 4096*4
MIN_SERVO_COMMAND = 410

EIGHT_BIT_NUMBERS_STORAGE = 256

# define Python user-defined exceptions
class InvalidNumberException(Exception):
    " "
    pass

def unnest_tuple(t):
    result = []
    for item in t:
        if isinstance(item, tuple):
            result.extend(unnest_tuple(item))
        else:
            result.append(item)
    return result
    
class UInt8():


    def __init__(self,value):
        self.UPPER_LIMIT = 256
        self.LOWER_LIMIT = 0 

        if value < self.LOWER_LIMIT or value > self.UPPER_LIMIT:
            raise InvalidNumberException(f"Integer has to be between {self.LOWER_LIMIT} and {self.UPPER_LIMIT}")
        self.value = value
    
    def float_to_array(self):
        first_byte = self.value
        return (first_byte)
    

class Int8():

    def __init__(self,value):
        self.UPPER_LIMIT = 128
        self.LOWER_LIMIT = -127

        if value < self.LOWER_LIMIT or value > self.UPPER_LIMIT:
            raise InvalidNumberException(f"Integer has to be between {self.LOWER_LIMIT} and {self.UPPER_LIMIT}")
        self.value = value
    
    def float_to_array(self):
        first_byte = self.value
        return (first_byte)
    
class Int16():

    def __init__(self,value : int):
        UPPER_LIMIT = 32767
        LOWER_LIMIT = -32768 
        if value < LOWER_LIMIT or value > UPPER_LIMIT:
            raise InvalidNumberException(f"Integer has to be between {LOWER_LIMIT} and {UPPER_LIMIT}")
        self.value = value
    
    def float_to_array(self):
        # Convert the float value to a 32-bit binary representation
        binary = struct.pack('h', self.value)

        # Unpack the binary representation into four 8-bit integers
        byte1, byte2= struct.unpack('2B', binary)

        # Return the list of integers
        return (byte1, byte2)

class Float():
    def __init__(self, value: float) -> None:
        self.value = value
        return
    
    def float_to_array(self):
        # Convert the float value to a 32-bit binary representation
        binary = struct.pack('f', self.value)

        # Unpack the binary representation into four 8-bit integers
        byte1, byte2, byte3, byte4 = struct.unpack('4B', binary)

        # Return the list of integers
        return (byte1, byte2, byte3, byte4)

class Buffer():
    def __init__(self, buffer:Tuple[Union[Float,Int16,Int8,UInt8]]) -> None:
        self.buffer = buffer
    
    def get_data(self):
        return (data.value for data in dataclasses.asdict(self.buffer).values())
    
    def get_checksum(self):
        checksum = []
        for data in dataclasses.asdict(self.buffer).values():
            checksum.append(data.float_to_array())
        unnested_checksum = unnest_tuple(checksum)
        return sum(unnested_checksum) % 256

@dataclass 
class TeensyPackage_out():
    arm_motors: int = 0
    arm_servos: int = 0
    motor_1_dshot_cmd: int = 0
    motor_2_dshot_cmd: int = 0
    motor_3_dshot_cmd: int = 0
    motor_4_dshot_cmd: int = 0
    servo_angle_1: int = 0
    servo_angle_2: int = 0
    servo_angle_3: int = 0
    servo_angle_4: int = 0
    servo_angle_5: int = 0
    servo_angle_6: int = 0
    servo_angle_7: int = 0
    servo_angle_8: int = 0
    servo_angle_9: int = 0
    servo_angle_10: int = 0
    servo_speed_1 : int = 0
    servo_speed_2: int = 0
    servo_speed_3: int = 0
    servo_speed_4: int = 0
    servo_mode_1: int = 0
    servo_mode_2: int = 0
    servo_mode_3: int = 0
    servo_mode_4: int = 0
    message: float = 0
    message_id: int = 0
    
    def __post_init__(self):

        self.arm_motors = Int8(self.arm_motors)
        self.arm_servos= Int8(self.arm_servos)
        self.motor_1_dshot_cmd = Int16(self.motor_1_dshot_cmd)
        self.motor_2_dshot_cmd = Int16(self.motor_2_dshot_cmd)
        self.motor_3_dshot_cmd = Int16(self.motor_3_dshot_cmd)
        self.motor_4_dshot_cmd = Int16(self.motor_4_dshot_cmd)
        self.servo_angle_1= Int16(self.servo_angle_1)
        self.servo_angle_2= Int16(self.servo_angle_2)
        self.servo_angle_3= Int16(self.servo_angle_3)
        self.servo_angle_4= Int16(self.servo_angle_4)
        self.servo_angle_5= Int16(self.servo_angle_5)
        self.servo_angle_6= Int16(self.servo_angle_6)
        self.servo_angle_7= Int16(self.servo_angle_7)
        self.servo_angle_8= Int16(self.servo_angle_8)
        self.servo_angle_9= Int16(self.servo_angle_9)
        self.servo_angle_10= Int16(self.servo_angle_10)
        self.servo_speed_1 = Int16(self.servo_speed_1)
        self.servo_speed_2= Int16(self.servo_speed_2)
        self.servo_speed_3= Int16(self.servo_speed_3)
        self.servo_speed_4= Int16(self.servo_speed_4)
        self.servo_mode_1= UInt8(self.servo_mode_1)
        self.servo_mode_2=  UInt8(self.servo_mode_2)
        self.servo_mode_3= UInt8(self.servo_mode_3)
        self.servo_mode_4= UInt8(self.servo_mode_4)
        self.message = Float(self.message)
        self.message_id= UInt8(self.message_id)


@dataclass
class TeensyPackage_in():
    motor_1_rpm : int
    motor_2_rpm : int
    motor_3_rpm : int
    motor_4_rpm : int
    motor_1_error_code : int
    motor_2_error_code : int
    motor_3_error_code : int
    motor_4_error_code : int
    motor_1_current : int
    motor_2_current : int
    motor_3_current : int
    motor_4_current : int
    motor_1_voltage : int
    motor_2_voltage : int
    motor_3_voltage : int
    motor_4_voltage : int
    servo_1_angle : int
    servo_2_angle : int
    servo_3_angle : int
    servo_4_angle : int
    servo_5_angle : int
    servo_6_angle : int
    servo_7_angle : int
    servo_8_angle : int
    servo_9_angle : int
    servo_10_angle : int
    servo_1_update_time_us : int
    servo_2_update_time_us : int
    servo_3_update_time_us : int
    servo_4_update_time_us : int
    servo_5_update_time_us : int
    servo_6_update_time_us : int
    servo_7_update_time_us : int
    servo_8_update_time_us : int
    servo_9_update_time_us : int
    servo_10_update_time_us : int
    rolling_msg_out : float
    rolling_msg_out_id : int
    checksum_out : int 

    def __post_init__(self):
        self.motor_1_rpm = Int16(self.motor_1_rpm)
        self.motor_2_rpm = Int16(self.motor_2_rpm)
        self.motor_3_rpm = Int16(self.motor_3_rpm)
        self.motor_4_rpm = Int16(self.motor_4_rpm)
        self.motor_1_error_code  = Int16(self.motor_1_error_code)
        self.motor_2_error_code  = Int16(self.motor_2_error_code)
        self.motor_3_error_code  = Int16(self.motor_3_error_code)
        self.motor_4_error_code  = Int16(self.motor_4_error_code)
        self.motor_1_current  = Int16(self.motor_1_current)
        self.motor_2_current  = Int16(self.motor_2_current)
        self.motor_3_current  = Int16(self.motor_3_current)
        self.motor_4_current  = Int16(self.motor_4_current)
        self.motor_1_voltage  = Int16(self.motor_1_voltage)
        self.motor_2_voltage  = Int16(self.motor_2_voltage)
        self.motor_3_voltage  = Int16(self.motor_3_voltage)
        self.motor_4_voltage  = Int16(self.motor_4_voltage)
        self.servo_1_angle  = Int16(self.servo_1_angle)
        self.servo_2_angle  = Int16(self.servo_2_angle)
        self.servo_3_angle  = Int16(self.servo_3_angle)
        self.servo_4_angle  = Int16(self.servo_4_angle)
        self.servo_5_angle  = Int16(self.servo_5_angle)
        self.servo_6_angle  = Int16(self.servo_6_angle)
        self.servo_7_angle  = Int16(self.servo_7_angle)
        self.servo_8_angle  = Int16(self.servo_8_angle)
        self.servo_9_angle  = Int16(self.servo_9_angle)
        self.servo_10_angle  = Int16(self.servo_10_angle)
        self.servo_1_update_time_us = Int16(self.servo_1_update_time_us)
        self.servo_2_update_time_us = Int16(self.servo_2_update_time_us)
        self.servo_3_update_time_us = Int16(self.servo_3_update_time_us)
        self.servo_4_update_time_us = Int16(self.servo_4_update_time_us)
        self.servo_5_update_time_us = Int16(self.servo_5_update_time_us)
        self.servo_6_update_time_us = Int16(self.servo_6_update_time_us )
        self.servo_7_update_time_us = Int16(self.servo_7_update_time_us)
        self.servo_8_update_time_us = Int16(self.servo_8_update_time_us)
        self.servo_9_update_time_us = Int16(self.servo_9_update_time_us)
        self.servo_10_update_time_us = Int16(self.servo_10_update_time_us)
        self.rolling_msg_out  = Float(self.rolling_msg_out)
        self.rolling_msg_out_id = UInt8(self.rolling_msg_out_id)
        self.checksum_out = UInt8(self.checksum_out)


    def get_checksum(self):
        checksum = []
        values = list(asdict(self).values())
        for data in values[:-1]: 
            # Get all values except for checksum
            checksum.append(data.float_to_array())
        unnested_checksum = unnest_tuple(checksum)
        return sum(unnested_checksum) % 256

def create_payload_package():
    return TeensyPackage_out()

def create_serial_bufer(payload):
    return Buffer(payload)

def set_servo_positions(serial_connection: serial.Serial, positions):
    payload = create_payload_package()
    payload.arm_servos.value = 1 

    for idx, position in enumerate(positions):
        servo_id = idx+1
        # Ensure that servo positions are within bounds
        # position = max(position, MIN_SERVO_COMMAND)
        # position = min(position,MAX_SERVO_COMMAND)
        getattr(payload,f'servo_angle_{servo_id}').value = position

        print(f"Setting position for servo {servo_id} to {position}")
        
    buffer = create_serial_bufer(payload)
    payload_out = struct.pack(struct_out, *buffer.get_data(), buffer.get_checksum())

    serial_connection.write(START_BYTE)
    serial_connection.write(payload_out)
    serial_connection.flush()
    sleep(0.1)


    return 
    
class Servo():

    
    def __init__(self, servo_id, init_angle=0):
        self.servo_id = servo_id
        self.trim_position = 0
        self._servo_angle = init_angle # deg * 100
        
    
    @property
    def servo_angle(self):
        return self._servo_angle
    
    @property
    def rotation_count(self):
        return self._get_rotation_count()

    def update_servo_angle(self, angle : Int16):
        change_in_angle = abs(self._servo_angle - angle)
        self.total_rotation += change_in_angle
        self._servo_angle = angle
        return
    
    def zero_servo(self, serial_connection):
        payload = create_payload_package()
        payload.arm_servos.value = 1 
        getattr(payload,f'servo_mode_{self.servo_id}').value = 0
        getattr(payload,f'servo_angle_{self.servo_id}').value = 0
        buffer = create_serial_bufer(payload)

        payload_out = struct.pack(struct_out, *buffer.get_data(), buffer.get_checksum())

        serial_connection.write(START_BYTE)
        serial_connection.write(payload_out)
        serial_connection.flush()
        sleep(0.1)
        return
    
    def set_servo_position(self, serial_connection: serial.Serial, position):
        payload = create_payload_package()
        payload.arm_servos.value = 1 
        getattr(payload,f'servo_mode_{self.servo_id}').value = 0
        getattr(payload,f'servo_angle_{self.servo_id}').value = position
        buffer = create_serial_bufer(payload)
        payload_out = struct.pack(struct_out, *buffer.get_data(), buffer.get_checksum())

        try:
            serial_connection.write(START_BYTE)
            serial_connection.write(payload_out)
            serial_connection.flush()

            print(f"Send position {position} to servo {self.servo_id}")
            sleep(0.1)
        except Exception as e:
            print(e)

        return 
    
    def _get_rotation_count(self):
        return floor(self.total_rotation / 36000)
    
START_BYTE = b'\x9A'
struct_out = "<2b18h4B1f2B"
struct_in = "<36h1f2B"
payload_in_size = struct.calcsize(struct_in)
        

init_servo = True
recieving_data = False
running = True 
rotation_count = 0

class MotorDriver(Node):

    def __init__(self, serial_connection):
        print("Init Node")
        super().__init__('mpr121_sub')
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            '/motor_position_reference',
            self.listener_callback,
            10)
        self.ser = serial_connection
        self.subscription  # prevent unused variable warning
        self.servo_1 = Servo(servo_id=1)
        self.servo_2 = Servo(servo_id=2)
        self.servo_3 = Servo(servo_id=3)

    def listener_callback(self, msg):
        if not self.ser.isOpen():
            self.ser.open()
        position_references = msg.data
        set_servo_positions(self.ser, position_references)

        self.ser.close()
        return

def main(args=None):
    rclpy.init(args=args)
    serial_connection = serial.Serial(port='/dev/ttyACM0', baudrate=150000,timeout=None, bytesize=serial.EIGHTBITS)
    sleep(0.2)
    motor_driver = MotorDriver(serial_connection)

    rclpy.spin(motor_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

       


