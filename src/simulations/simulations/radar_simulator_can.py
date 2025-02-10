import random

import can
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import SimulatedObject


class RadarSimulator(Node):

    def __init__(self):
        super().__init__('radar_simulator')
        self.object_subscription = self.create_subscription(
            SimulatedObject,
            '/simulated_objects',
            self.send_radar_data,
            10)
        self.bus = can.interface.Bus(channel='vcan0', bustype='socketcan')

    def send_radar_data(self, simulated_object):
        """Send radar data frames over the virtual CAN bus."""
        # Generate radar data
        vrel_long = random.randint(-30, 30)  # Relative velocity in m/s
        cluster_id = simulated_object.object_id  # Cluster ID: 0-255 (8 bits)
        dist_long = simulated_object.position.x  # Longitudinal distance in meters
        dist_lat = simulated_object.position.y   # Lateral distance in meters
        height = simulated_object.position.z     # Height in meters

        # Format the CAN frame
        frame = self.format_radar_frame(cluster_id, dist_long, dist_lat, vrel_long, height)
        msg = can.Message(arbitration_id=0x701, data=frame, is_extended_id=False)

        try:
            self.bus.send(msg)
            self.get_logger().info(f'Sent Radar Frame: ID={cluster_id}')
        except can.CanError as e:
            self.get_logger().info(f'Failed to send frame: {e}')

    @staticmethod
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


def main(args=None):
    rclpy.init(args=args)
    radar_simulator = RadarSimulator()
    rclpy.spin(radar_simulator)
    radar_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
