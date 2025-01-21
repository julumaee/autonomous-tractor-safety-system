import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import can

class RadarPublisher(Node):

    def __init__(self):
        super().__init__('radar_publisher')
        self.publisher_ = self.create_publisher(String, 'radar_detections', 10)
        # Set up the CAN interface with vcan0 and socketcan
        self.bus = can.interface.Bus(channel = 'vcan0', bustype='socketcan')
        self.listen_to_can()

    def listen_to_can(self):
        # Read and publish CAN messages in a loop
        while rclpy.ok():
            message = self.bus.recv(timeout=1) # Blocking call, waits for a message for a second
            if message:
                msg = self.process_radar_data(message)
                self.publisher_.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)
    

    # TODO This processing needs to be re-configured after determining the correct message format for radar

    def process_radar_data(self, message):
        # Extract data from CAN message
        distance =  (message.data[0] | (message.data[1] << 8))
        angle = (message.data[2] | (message.data[3] << 8)) # Can be negative, so handle as signed
        if angle & (1 << 15):  # Check the sign bit
            angle -= (1 << 16)
        speed = (message.data[4] | (message.data[5] << 8)) # Can be negative, so handle as signed
        if speed & (1 << 15):  # Check the sign bit
            speed -= (1 << 16)
        counter =   (message.data[6] | (message.data[7] << 8))

        msg = String()
        msg.data = f"Radar data: Distance={distance}m, Angle={angle}°, Speed={speed}m/s, Message nr.={counter}"
        return msg


def main(args=None):
    rclpy.init(args=args)
    radar_publisher = RadarPublisher()
    rclpy.spin(radar_publisher)
    radar_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()