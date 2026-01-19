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

import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import (
    BoundingBox2D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)

from tractor_safety_system_interfaces.msg import SimulatedObject


class CameraSimulator(Node):

    def __init__(self):
        super().__init__("camera_simulator")
        self.publisher_ = self.create_publisher(
            Detection3DArray, "/oak/nn/spatial_detections", 10
        )
        self.object_subscription = self.create_subscription(
            SimulatedObject, "/simulated_objects", self.simulate_detection, 10
        )

    def simulate_detection(self, simulated_object):
        """Simulate detections and publish them as a SpatialDetectionArray message."""
        # Create a simulated Detection3DArray message with one detection
        distance = (
            simulated_object.position.x**2 + simulated_object.position.y**2
        ) ** 0.5
        if distance > 15:
            self.get_logger().info(f"Object is too far away: {distance}m")
        else:
            detection_array = Detection3DArray()
            detection_array.header = Header()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = "oakd_camera_frame"

            detection = Detection3D()
            detection.results = []
            # Create ObjectHypothesisWithPose from ObjectHypothesis
            result_with_pose = ObjectHypothesisWithPose()
            result_with_pose.hypothesis = self.generate_object_hypothesis()
            # Set pose position to match the detection position
            camera_position = Point(
                x=-simulated_object.position.y,
                y=-simulated_object.position.z,
                z=simulated_object.position.x,
            )
            result_with_pose.pose.pose.position = camera_position
            detection.results = [result_with_pose]
            
            detection.bbox = (
                self.generate_bounding_box()
            )
            
            detection.is_tracking = random.choice([True, False])
            if detection.is_tracking:
                detection.tracking_id = f"object_{simulated_object.object_id}"

            # Add detection to the array
            detection_array.detections.append(detection)

            # Publish the message
            self.publisher_.publish(detection_array)
            self.get_logger().info(
                f"Published a detection as {detection_array.header.frame_id}"
            )

    @staticmethod
    def generate_object_hypothesis():
        """Generate a simulated object hypothesis."""
        hypothesis = ObjectHypothesis(
            class_id="person", score=random.uniform(0.7, 0.95)
        )
        return hypothesis

    @staticmethod
    def generate_bounding_box():
        """Generate a random bounding box."""
        bbox = BoundingBox2D()
        bbox.center.position.x = random.uniform(100, 500)
        bbox.center.position.y = random.uniform(100, 500)
        bbox.center.theta = 0.0
        bbox.size_x = random.uniform(50, 150)
        bbox.size_y = random.uniform(50, 150)
        return bbox


def main(args=None):
    rclpy.init(args=args)
    camera_simulator = CameraSimulator()
    rclpy.spin(camera_simulator)
    camera_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
