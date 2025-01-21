import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# Serial port configuration
UART_PORT = '/dev/pts/4' # Adjust to match the port used!
BAUD_RATE = 115200

class RadarReceiver(Node):
    def __init__(self):
        super().__init__('radar_receiver')
        self.publisher_ = self.create_publisher(String, '/detections', 10)
        self.serial_port = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
        #self.read_uart() # If we wan't to read UART with no delay, we need to implement a multithreaded call for this...
        self.timer = self.create_timer(0.1, self.read_uart)

    def read_uart(self):
        """Read data from the UART interface."""
        # while rclpy.ok(): # If we wan't to read UART with no delay, we need to implement a multithreaded call for this...
        if self.serial_port.in_waiting > 0:
            frame = self.serial_port.read(self.serial_port.in_waiting)
            self.process_frame(frame)

    def process_frame(self, frame):
        """Process and decode the received UART frame."""
        if len(frame) < 10:  # Minimum frame length
            return
        if frame[:2] != b'\xAA\xAA' or frame[-2:] != b'\x55\x55':  # Check start and end codes of SR75 UART frame format
            return
        frame_id = frame[2] | (frame[3] << 8)
        distance = frame[4] | (frame[5] << 8)
        angle = frame[6] | (frame[7] << 8) # Can be negative, handle as signed!
        if angle & (1 << 15):  # Check the sign bit
            angle -= (1 << 16)
        speed = frame[8] | (frame[9] << 8) # Can be negative, handle as signed!
        if speed & (1 << 15):  # Check the sign bit
            speed -= (1 << 16)
        timestamp = self.get_clock().now().to_msg()
        msg = String()
        msg.data = f"Radar detection at time {timestamp.sec}.{timestamp.nanosec}: Frame ID: {frame_id}, Distance: {distance}, Angle: {angle}, Speed: {speed}"
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    radar_receiver = RadarReceiver()
    rclpy.spin(radar_receiver)
    radar_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()