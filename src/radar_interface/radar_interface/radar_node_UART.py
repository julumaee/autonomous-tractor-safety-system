import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import RadarDetection


# Serial port configuration
UART_PORT = '/dev/pts/6' # Adjust to match the port used!
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
        if len(frame) < 10:  # Minimum frame length
            return
        if frame[:2] != b'\xAA\xAA' or frame[-2:] != b'\x55\x55':  # Check start and end codes of SR75 UART frame format
            return
        # Create a RadarDetection with correct information
        radar_detection_msg = RadarDetection()
        radar_detection_msg.header = Header()
        radar_detection_msg.header.frame_id = f"target_{frame[2] | (frame[3] << 8)}"
        radar_detection_msg.header.stamp = self.get_clock().now().to_msg()
        radar_detection_msg.distance = frame[4] | (frame[5] << 8)
        radar_detection_msg.angle = frame[6] | (frame[7] << 8) # Can be negative, handle as signed!
        if radar_detection_msg.angle & (1 << 15):  # Check the sign bit
            radar_detection_msg.angle -= (1 << 16)
        radar_detection_msg.speed = frame[8] | (frame[9] << 8) # Can be negative, handle as signed!
        if radar_detection_msg.speed & (1 << 15):  # Check the sign bit
            radar_detection_msg.speed -= (1 << 16)

        self.publisher_.publish(radar_detection_msg)
        self.get_logger().info(f"Publishing radar detection with id: {radar_detection_msg.header.frame_id}")

def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarNode()
    rclpy.spin(radar_node)
    radar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()