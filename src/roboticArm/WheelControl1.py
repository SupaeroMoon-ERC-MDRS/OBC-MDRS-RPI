import serial
import time

# RoboClaw settings
SERIAL_PORT = "/dev/ttyS0" # dmesg | grep tty to get serial connections
BAUDRATE = 115200
ADDRESS = [0x80, 0x81, 0x82]

# Basic commands
FORWARD_M1 = 0
FORWARD_M2 = 4
BACKWARD_M1 = 1
BACKWARD_M2 = 5
STOP_COMMAND = 0x12

# Initialize serial port
def initialize_serial(port, baudrate):
    ser = serial.Serial(port, baudrate, timeout=1)
    if ser.is_open:
        print(f"Serial port {port} initialized.")
    else:
        raise Exception("Failed to open serial port.")
    return ser

# Send command to RoboClaw
def send_command(serial_connection, address, command, value):
    try:
        packet = bytearray([address, command, value])
        checksum = sum(packet) & 0x7F
        packet.append(checksum)
        serial_connection.write(packet)
        response = serial_connection.read(1)
        if response:
            print(f"Response: {response.hex()}")
    except Exception as e:
        print(f"Error sending command: {e}")

# Placeholder for joystick input logic
def get_joystick_input():
    """
    Replace this function with actual joystick input logic.
    Returns a list of values [(M1, M2), (M1, M2), (M1, M2)].
    """
    return [(50, 50), (0, 0), (-50, -50)]

# Calculate wheel speeds based on joystick input
def calculate_wheel_speeds(joystick_x, joystick_y):
    left_speed = joystick_y + joystick_x
    right_speed = joystick_y - joystick_x
    left_speed = max(min(left_speed, 127), -127)
    right_speed = max(min(right_speed, 127), -127)
    return [(left_speed, right_speed)] * 3

# Control rover using joystick input
def control_rover(serial_connection, joystick_input):
    for i, addr in enumerate(ADDRESS):
        if joystick_input[i][0] > 0:
            send_command(serial_connection, addr, FORWARD_M1, joystick_input[i][0])
        elif joystick_input[i][0] < 0:
            send_command(serial_connection, addr, BACKWARD_M1, abs(joystick_input[i][0]))

        if joystick_input[i][1] > 0:
            send_command(serial_connection, addr, FORWARD_M2, joystick_input[i][1])
        elif joystick_input[i][1] < 0:
            send_command(serial_connection, addr, BACKWARD_M2, abs(joystick_input[i][1]))

# Main function
def main():
    try:
        serial_connection = initialize_serial(SERIAL_PORT, BAUDRATE)
        print("Rover control ready.")

        while True:
            joystick_x = int(input("Joystick X (-127 to 127): "))
            joystick_y = int(input("Joystick Y (-127 to 127): "))

            joystick_input = calculate_wheel_speeds(joystick_x, joystick_y)

            control_rover(serial_connection, joystick_input)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping the rover...")
        for addr in ADDRESS:
            send_command(serial_connection, addr, STOP_COMMAND, 0)
        serial_connection.close()
        print("Rover stopped and connection closed.")

if __name__ == "__main__":
    main()
