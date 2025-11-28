# Copyright 2025 Eemil Kulmala, University of Oulu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from depthai_ros_msgs.msg import SpatialDetectionArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from tractor_safety_system_interfaces.msg import CameraDetection


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(CameraDetection, '/camera_detections', 10)
        self.subscription = self.create_subscription(
            SpatialDetectionArray,
            # The topic name should match the one in the launch file
            '/color/yolov4_Spatial_detections',
            self.publish_detections,
            10)
        self.subscription  # prevent unused variable warning

    def convert_message(self, detection):
        """Convert messages from SpatialDetection to CameraMessage format."""
        camera_detection_msg = CameraDetection()
        camera_detection_msg.results = []
        for result in detection.results:
            camera_detection_msg.results.append(result)
        camera_detection_msg.bbox = detection.bbox
        camera_detection_msg.position = detection.position
        camera_detection_msg.header = Header()
        camera_detection_msg.header.stamp = self.get_clock().now().to_msg()
        camera_detection_msg.header.frame_id = 'camera_link'
        return camera_detection_msg

    def publish_detections(self, oakd_msg):
        """Publish detections as individual CameraDetections from a SpatialDetectionArray."""
        for detection in oakd_msg.detections:
            camera_detection_msg = self.convert_message(detection)
            self.publisher_.publish(camera_detection_msg)
            # self.get_logger().info('Publishing camera detection at: '
            #                       f'{camera_detection_msg.position.x:.2f}, '
            #                       f'{camera_detection_msg.position.y:.2f}, '
            #                       f'{camera_detection_msg.position.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
