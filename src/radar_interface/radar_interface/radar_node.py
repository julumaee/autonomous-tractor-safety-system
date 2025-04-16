import can
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import RadarDetection


class RadarNode(Node):

    def __init__(self):
        super().__init__('radar_publisher')
        self.publisher_ = self.create_publisher(RadarDetection, '/radar_detections', 10)
        self.declare_parameter('can_channel', 'can0')
        self.can_channel = self.get_parameter('can_channel').value
        
        # Set up the CAN interface with correct can channel and socketcan
        try:
            self.bus = can.interface.Bus(channel=self.can_channel, interface='socketcan')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CAN bus: {e}')
            return
        self.get_logger().info(f'Connected to CAN bus on channel {self.can_channel}')

        #self.send_pointcloud_config()  # Not working correctly

        self.frame_buffer = {}
        self.listen_to_can()

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == 'can_channel':
                self.can_channel = param.value
        return SetParametersResult(successful=True)

    def send_pointcloud_config(self):
        """Send configuration to radar to enable point cloud mode."""
        try:
            # Payload: enable RadarCfg_OutputType = 0x02 (point cloud / clusters), store in NVM
            data=[
                0b00001000,
                0b00000000,
                0b00000000,
                0b00000000,
                0b00010000,
                0b00000000,
                0b00000000,
                0b00000000
            ]
            msg = can.Message(arbitration_id=0x200, data=data, is_extended_id=False)
            self.bus.send(msg)
            self.get_logger().info("Sent radar configuration: Point Cloud mode (clusters)")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send radar config: {e}")

    def listen_to_can(self):
        # Read and publish CAN messages in a loop
        while rclpy.ok():
            frame = self.bus.recv(timeout=0.1)  # Waits for a message for 0.1 seconds
            if frame:
                self.process_radar_data(frame)

    def process_radar_data(self, frame):
        message_id = frame.arbitration_id
        data = frame.data

        if message_id != 0x701 or len(data) != 8:
            return

        target_id = data[0] & 0x7F
        frame_number = (data[0] & 0x80) >> 7

        if frame_number == 0:
            # Store first part of the trace
            self.frame_buffer[target_id] = {'frame0': data}
        elif frame_number == 1:
            # Store second part of the trace and try to combine
            if target_id in self.frame_buffer and 'frame0' in self.frame_buffer[target_id]:
                self.frame_buffer[target_id]['frame1'] = data
                self.publish_radar_detection(target_id)

    def publish_radar_detection(self, target_id):
        try:
            frame0 = self.frame_buffer[target_id]['frame0']
            frame1 = self.frame_buffer[target_id]['frame1']
            del self.frame_buffer[target_id]  # Clean up buffer

            # Longitudinal distance
            dist_long = ((frame0[1] << 5) | (frame0[2] >> 3)) & 0x1FFF
            dist_long = dist_long * 0.05 - 100

            # Lateral distance
            dist_lat = (((frame0[2] & 0x07) << 8) | frame0[3]) * 0.05 - 50

            distance = (dist_long ** 2 + dist_lat ** 2) ** 0.5
            # self.get_logger().info(f'Distance={distance:.2f}')  # Debugging info

            # Longitudinal speed
            vrel_long = ((frame0[4] << 2) | (frame0[5] >> 6)) & 0x3FF
            vrel_long = vrel_long * 0.25 - 128

            # Height from frame1
            height = ((frame1[1] << 2) | (frame1[2] >> 6)) & 0x3FF
            height = height * 0.1 - 30

            # Publish message
            radar_detection_msg = RadarDetection()
            radar_detection_msg.header = Header()
            radar_detection_msg.header.frame_id = f'target_{target_id}'
            radar_detection_msg.header.stamp = self.get_clock().now().to_msg()

            radar_detection_msg.position.x = dist_long
            radar_detection_msg.position.y = dist_lat
            radar_detection_msg.position.z = height
            radar_detection_msg.speed = int(vrel_long)
            radar_detection_msg.distance = int((dist_long ** 2 + dist_lat ** 2) ** 0.5)

            self.publisher_.publish(radar_detection_msg)
            self.get_logger().info(
                f'Radar target ID={target_id}, Distance={radar_detection_msg.distance:.2f}, '
                f'Speed={vrel_long:.2f} m/s'
            )

        except Exception as e:
            self.get_logger().error(f'Error publishing radar detection: {e}')


def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarNode()
    rclpy.spin(radar_node)
    radar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
