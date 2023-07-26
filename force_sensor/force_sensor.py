import serial
from time import sleep, time
from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian

unlock_command = 0x18
start_streaming_command = 0x55
jamming_sequence = [0xFF] * 14
# Configure serial port settings
serial_port = "COM9"  # Replace with the correct serial port
baud_rate = 115200 # Set the baud rate to 1.25 megabaud
parity = serial.PARITY_EVEN  # Set the parity to even

starting_address = 10  # Replace with the starting address of the registers
# Create Modbus client
client = ModbusSerialClient(method='rtu', port=serial_port, baudrate=baud_rate, parity=parity)


try:
    # Connect to the sensor
    if not client.connect():
        print("Failed to connect to the sensor")
        exit(1)
    print("Connected to sensor")
    # Read force and torque values
    client.write_registers(starting_address,unlock_command)
    client.write_registers(starting_address,start_streaming_command)
    sleep(0.002)
    time_start = time()
    time_now = time()
    while time_now - time_start < 2:
        reponse = client.read_holding_registers(starting_address)
        print(reponse)
        time_now = time()


    client.write_registers(starting_address,jamming_sequence)

finally:
    client.close()
    print("stopped")
