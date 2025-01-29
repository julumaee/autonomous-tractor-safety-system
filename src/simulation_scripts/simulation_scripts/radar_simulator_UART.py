import time
import random
import serial
from struct import pack

# Virtual UART port
UART_PORT = '/dev/pts/4' # Adjust to match the port used!
BAUD_RATE = 115200

def generate_radar_data():
    """Generate simulated radar data."""
    target_id = random.randint(0, 255)         # Target ID: 0-255
    distance_long = random.uniform(-100, 100)  # Longitudinal distance in meters
    distance_lat = random.uniform(-50, 50)     # Lateral distance in meters
    velocity = random.uniform(-30, 30)         # Velocity in m/s
    height = 0                                 # Ground plane
    return target_id, distance_long, distance_lat, velocity, height

def send_radar_data():
    """Send radar data frames over the virtual UART."""
    with serial.Serial(UART_PORT, BAUD_RATE) as ser:
        while True:
            target_id, distance_long, distance_lat, velocity, height = generate_radar_data()
            frame = format_uart_frame(target_id, distance_long, distance_lat, velocity, height)
            ser.write(frame)
            print(f"Sent data: Target ID: {target_id}, Long: {distance_long:.2f}, Lat: {distance_lat:.2f}, Vel: {velocity:.2f}, Height: {height:.2f}")
            time.sleep(2)  # Send data every 2s

def format_uart_frame(target_id, distance_long, distance_lat, velocity, height):
    """Format a UART data frame based on the protocol."""
    # Scale and encode fields
    long_scaled = int((distance_long + 100) / 0.05) & 0xFFFF  # Longitudinal distance
    lat_scaled = int((distance_lat + 50) / 0.05) & 0xFFFF     # Lateral distance
    vel_scaled = int((velocity + 128) / 0.25) & 0xFFFF        # Velocity
    height_scaled = int((height + 64) / 0.25) & 0xFFFF        # Height

    # Pack the fields into a message (big-endian format)
    payload = pack('>BHHHH', target_id, long_scaled, lat_scaled, vel_scaled, height_scaled)

    # Add start and end bytes
    frame = b'\xAA\xAA' + payload + b'\x55\x55'
    return frame


if __name__ == '__main__':
    send_radar_data()