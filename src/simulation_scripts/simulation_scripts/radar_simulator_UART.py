import time
import random
import serial

# Virtual UART port
UART_PORT = '/dev/pts/5' # Adjust to match the port used!
BAUD_RATE = 115200

def generate_radar_data():
    """Generate simulated radar data."""
    distance = random.randint(5, 100)  # Distance in meters
    angle = random.randint(-30, 30)   # Angle in degrees
    speed = random.randint(-30, 30)   # Speed in m/s
    return distance, angle, speed

def format_uart_frame(frame_id, distance, angle, speed):
    """Format a UART data frame based on the protocol."""
    payload = [
        distance & 0xFF, (distance >> 8) & 0xFF,
        angle & 0xFF, (angle >> 8) & 0xFF,
        speed & 0xFF, (speed >> 8) & 0xFF
    ]
    frame = [0xAA, 0xAA]  # Start Code
    frame += [frame_id & 0xFF, (frame_id >> 8) & 0xFF]  # Frame ID
    frame += payload  # Payload
    frame += [0x55, 0x55]  # End Code
    return bytearray(frame)

def send_radar_data():
    """Send radar data frames over the virtual UART."""
    frame_id = 0x01  # Example frame ID
    with serial.Serial(UART_PORT, BAUD_RATE) as ser:
        while True:
            distance, angle, speed = generate_radar_data()
            frame = format_uart_frame(frame_id, distance, angle, speed)
            ser.write(frame)
            #print(f"Sent frame: {frame.hex()}") # Print sent data as hex
            print(f"Sent data: Frame ID: {frame_id}, Distance: {distance}, Angle: {angle}, Speed: {speed}")
            frame_id += 1
            time.sleep(2)  # Send data every 2s

if __name__ == '__main__':
    send_radar_data()