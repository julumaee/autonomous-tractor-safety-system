import random
import time

import can


def generate_radar_data():
    """Generate simulated radar data."""
    cluster_id = random.randint(0, 255)       # Cluster ID: 0-255 (8 bits)
    dist_long = random.randint(0, 50)         # Longitudinal distance in meters
    dist_lat = random.randint(-50, 50)        # Lateral distance in meters
    vrel_long = random.randint(-30, 30)       # Relative velocity in m/s
    height = 0.0
    return cluster_id, dist_long, dist_lat, vrel_long, height


def send_radar_data():
    """Send radar data frames over the virtual CAN bus."""
    bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

    while True:
        # Generate radar data
        cluster_id, dist_long, dist_lat, vrel_long, height = generate_radar_data()

        # Format the CAN frame
        frame = format_radar_frame(cluster_id, dist_long, dist_lat, vrel_long, height)
        msg = can.Message(arbitration_id=0x701, data=frame, is_extended_id=False)

        try:
            bus.send(msg)
            print(f'Sent Radar Frame: ID={cluster_id}, \
                  Long={dist_long}m, \
                  Lat={dist_lat}m, \
                  Vel={vrel_long}m/s, \
                  Height={height}m')
        except can.CanError as e:
            print(f'Failed to send frame: {e}')

        time.sleep(2)  # Send data every 2 seconds


def format_radar_frame(cluster_id, dist_long, dist_lat, vrel_long, height):
    """Format a radar CAN frame based on the specified bit layout."""
    # Scale and encode fields
    dist_long_scaled = int((dist_long + 100) / 0.05) & 0x1FFF  # 13 bits
    dist_lat_scaled = int((dist_lat + 50) / 0.05) & 0x07FF     # 11 bits
    vrel_long_scaled = int((vrel_long + 128) / 0.25) & 0x03FF  # 10 bits
    height_scaled = int((height + 64) / 0.25) & 0x03FF         # 10 bits
    dyn_prop = 0x00

    # Pack the data into bytes according to the memory layout
    frame = bytearray(8)

    # Byte 0: Cluster_ID
    frame[0] = cluster_id & 0xFF

    # Bytes 1-2: DistLong (13 bits)
    frame[1] = (dist_long_scaled >> 5) & 0xFF
    frame[2] = ((dist_long_scaled & 0x1F) << 3) & 0xF8

    # Add DistLat (11 bits)
    frame[2] |= (dist_lat_scaled >> 8) & 0x07
    frame[3] = dist_lat_scaled & 0xFF

    # Bytes 4-5: VrelLong (10 bits) and Height (10 bits)
    frame[4] = (vrel_long_scaled >> 2) & 0xFF
    frame[5] = ((vrel_long_scaled & 0x03) << 6) & 0xC0
    frame[5] |= (height_scaled >> 4) & 0x3F

    # Bytes 6-7: Height (remaining 6 bits) dynProp (3 bits) and RCS (8 bits)
    frame[6] = ((height_scaled & 0x0F) << 4) & 0xF0  # Height (remaining bits)
    frame[6] |= (dyn_prop << 1) & 0x0E  # DynProp (3 bits, shifted left 1)

    frame[7] = 0x00  # RCS set to 0

    return bytes(frame)


if __name__ == '__main__':
    send_radar_data()
