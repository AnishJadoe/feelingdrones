from pymodbus.client import ModbusSerialClient

# Configure serial port settings
serial_port = '/dev/ttyUSB0'  # Replace with the correct serial port
baud_rate = 1250000  # Set the baud rate to 1.25 megabaud
parity = 'E'  # Set the parity to even

# Create Modbus client
client = ModbusSerialClient(method='rtu', port=serial_port, baudrate=baud_rate, parity=parity)

try:
    # Connect to the sensor
    if not client.connect():
        print("Failed to connect to the sensor")
        exit(1)
    print("Connected to sensor")
    # Read force and torque values
    starting_address = 10  # Replace with the starting address of the registers

    response = client.read_holding_registers(starting_address)

    if response.isError():
        print("Modbus error:", response)
    else:
        registers = response.registers

        # Extract and process force and torque values
        force_x = registers[0] / 10.0  # Assuming force_x is stored in the first register (adjust accordingly)
        force_y = registers[1] / 10.0  # Assuming force_y is stored in the second register (adjust accordingly)
        torque_z = registers[3] / 100.0  # Assuming torque_z is stored in the fourth register (adjust accordingly)

        # Print the values
        print("Force X:", force_x)
        print("Force Y:", force_y)
        print("Torque Z:", torque_z)

finally:
    # Close the Modbus connection
    client.close()