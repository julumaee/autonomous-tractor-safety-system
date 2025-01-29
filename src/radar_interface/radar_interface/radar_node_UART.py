import rclpy
import serial
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import RadarDetection
from struct import unpack


# Serial port configuration
UART_PORT = '/dev/pts/5' # Adjust to match the port used!
BAUD_RATE = 115200

class RadarNode(Node):
    def __init__(self):
        super().__init__('radar_node')
        self.publisher_ = self.create_publisher(RadarDetection, '/radar_detections', 10)
        self.serial_port = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
        self.timer = self.create_timer(0.1, self.read_uart)

    def read_uart(self):
        """Read data from the UART interface."""
        if self.serial_port.in_waiting > 0:
            frame = self.serial_port.read(self.serial_port.in_waiting)
            self.process_frame(frame)

    def process_frame(self, frame):
        """Process and decode the received UART frame."""

        # Validate start and end bytes
        if frame[:2] != b'\xAA\xAA' or frame[-2:] != b'\x55\x55':
            self.get_logger().warning("Invalid frame format: Missing start or end bytes.")
            return
        
        payload = frame[2:-2] # Extract the relevant data
        target_id, long_scaled, lat_scaled, vel_scaled, height_scaled = unpack('>BHHHH', payload)

        # Decode scaled values back to their original units
        distance_long = (long_scaled * 0.05) - 100  # Longitudinal distance
        distance_lat = (lat_scaled * 0.05) - 50     # Lateral distance
        velocity = (vel_scaled * 0.25) - 128        # Velocity
        height = (height_scaled * 0.25) - 64        # Height

        # Create and populate the RadarDetection message
        radar_detection_msg = RadarDetection()
        radar_detection_msg.header = Header()
        radar_detection_msg.header.frame_id = f"target_{target_id}"
        radar_detection_msg.header.stamp = self.get_clock().now().to_msg()
        radar_detection_msg.position.y = distance_long
        radar_detection_msg.position.x = distance_lat
        radar_detection_msg.position.z = height
        radar_detection_msg.speed = int(velocity)
        radar_detection_msg.distance = int(np.linalg.norm(
        [radar_detection_msg.position.x, radar_detection_msg.position.y, radar_detection_msg.position.z]
        ))

        self.publisher_.publish(radar_detection_msg)
        self.get_logger().info(
            f"Publishing radar detection: Target ID={target_id}, Long={distance_long:.2f}m, Lat={distance_lat:.2f}m, Vel={velocity:.2f}m/s"
        )
        #self.get_logger().info(f"Publishing radar detection with id: {radar_detection_msg.header.frame_id}")


def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarNode()
    rclpy.spin(radar_node)
    radar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()