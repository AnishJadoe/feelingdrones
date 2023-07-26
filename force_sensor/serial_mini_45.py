import serial

unlock_command = 0x18
start_streaming_command = 0x55
jamming_sequence = [0xFF] * 14

# Configure serial port settings
serial_port = "COM9"  # Replace with the correct serial port
baud_rate = 115200 # Set the baud rate to 1.25 megabaud 
parity = serial.PARITY_EVEN  # Set the parity to even

connection = serial.Serial(port=serial_port,baudrate=baud_rate,parity=parity)

connection.write(b'0x55')
print(connection.readlines(45))
connection.close()