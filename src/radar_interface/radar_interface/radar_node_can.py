import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import RadarDetection


class RadarNode(Node):

    def __init__(self):
        super().__init__('radar_publisher')
        self.publisher_ = self.create_publisher(RadarDetection, '/radar_detections', 10)
        # Set up the CAN interface with vcan0 and socketcan
        self.bus = can.interface.Bus(channel='vcan0', interface='socketcan')
        self.frame_buffer = {}
        self.listen_to_can()

    def listen_to_can(self):
        # Read and publish CAN messages in a loop
        while rclpy.ok():
            frame = self.bus.recv(timeout=0.1)  # Waits for a message for 0.1 seconds
            if frame:
                self.process_radar_data(frame)

    def process_radar_data(self, frame):
        """Process and decode radar (SR75) data based on the new format."""
        try:
            # Extract arbitration ID and data payload
            message_id = frame.arbitration_id
            data = frame.data

            # Check if the message ID matches target information (0x701)
            if message_id == 0x701:
                if len(data) != 8:
                    self.get_logger().warning('Invalid frame length. Expected 8 bytes.')
                    return

                # Parse the data based on the memory layout
                cluster_id = data[0]  # Byte 0: Cluster_ID

                # Bytes 1-2: DistLong (13 bits)
                dist_long = ((data[1] << 5) | (data[2] >> 3)) & 0x1FFF
                dist_long = (dist_long * 0.05) - 100

                # Bytes 2-3: DistLat (11 bits)
                dist_lat = ((data[2] & 0x07) << 8) | data[3]
                dist_lat = (dist_lat * 0.05) - 50

                # Bytes 4-5: VrelLong (10 bits)
                vrel_long = ((data[4] << 2) | (data[5] >> 6)) & 0x03FF
                vrel_long = (vrel_long * 0.25) - 128

                # Bytes 5-6: Height (10 bits) and dyn_prop (3)
                height = ((data[5] & 0x3F) << 4) | (data[6] >> 4)
                height = (height * 0.25) - 64

                # dyn_prop = (data[6] >> 1) & 0x07  # dyn_prop, currently not used
                # rcs = data[7]  # RCS, currently not used

                # Construct and publish RadarDetection message
                radar_detection_msg = RadarDetection()
                radar_detection_msg.header = Header()
                radar_detection_msg.header.frame_id = f'target_{cluster_id}'
                radar_detection_msg.header.stamp = self.get_clock().now().to_msg()
                radar_detection_msg.position.x = dist_long
                radar_detection_msg.position.y = dist_lat
                radar_detection_msg.position.z = height
                radar_detection_msg.speed = int(vrel_long)
                radar_detection_msg.distance = int(
                    (dist_long ** 2 + dist_lat ** 2 + height ** 2) ** 0.5
                )

                self.publisher_.publish(radar_detection_msg)
                self.get_logger().info(
                    f'Published RadarDetection: ID={cluster_id}, '
                    f'Distance={radar_detection_msg.distance}, '
                    f'Speed={vrel_long:.2f}m/s'
                )
            else:
                self.get_logger().debug(f'Received message with unknown ID: {message_id}')

        except Exception as e:
            self.get_logger().error(f'Error processing radar data: {e}')


def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarNode()
    rclpy.spin(radar_node)
    radar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
