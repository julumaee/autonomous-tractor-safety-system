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

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray, ObjectHypothesis

from tractor_safety_system_interfaces.msg import CameraDetection


class CameraNode(Node):
    """
    Camera interface node for OAK-D S2 camera with depthai-ros-driver.

    Subscribes to Detection3DArray messages from depthai-ros-driver and
    converts them to CameraDetection messages for use by the sensor fusion pipeline.

    Default input topic: /oak/nn/spatial_detections
    (when depthai-ros-driver is configured with neural network enabled)

    This can be configured via:
    - parameters.yaml file (camera_node.ros__parameters.input_topic)
    - Launch arguments (when launched directly)
    - Runtime parameter updates
    """

    def __init__(self):
        """Initialize the Camera Node."""
        super().__init__("camera_node")

        # Declare parameters with default values
        # Default input_topic matches depthai-ros-driver with neural network enabled:
        # /<camera_name>/nn/spatial_detections (e.g., /oak/nn/spatial_detections)
        # Can be overridden via parameters.yaml or launch arguments
        self.declare_parameter("input_topic", "/oak/nn/spatial_detections")
        self.declare_parameter("output_topic", "/camera_detections")
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("enable_logging", False)

        # Retrieve parameter values
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        queue_size = self.get_parameter("queue_size").value
        self.enable_logging = self.get_parameter("enable_logging").value

        # Create publisher
        self.publisher_ = self.create_publisher(
            CameraDetection, output_topic, queue_size
        )

        # Create subscription
        self.subscription = self.create_subscription(
            Detection3DArray, input_topic, self.publish_detections, queue_size
        )

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.get_logger().info(
            f"Camera node initialized. Subscribed to: {input_topic}, "
            f"Publishing to: {output_topic}, Frame ID: {self.frame_id}"
        )

    def on_set_parameters(self, params):
        """Handle parameter updates at runtime."""
        for param in params:
            if param.name == "frame_id":
                self.frame_id = param.value
                self.get_logger().info(f"Updated frame_id to: {self.frame_id}")
            elif param.name == "enable_logging":
                self.enable_logging = param.value
                self.get_logger().info(
                    f'Logging {"enabled" if self.enable_logging else "disabled"}'
                )
        return SetParametersResult(successful=True)

    def convert_message(self, detection):
        """Convert a Detection3D to CameraDetection format."""
        try:
            camera_detection_msg = CameraDetection()
            camera_detection_msg.header = Header()
            camera_detection_msg.header.stamp = self.get_clock().now().to_msg()
            camera_detection_msg.header.frame_id = self.frame_id

            # Extract ObjectHypothesis from ObjectHypothesisWithPose
            if hasattr(detection, "results") and detection.results:
                camera_detection_msg.results = []
                for result_with_pose in detection.results:
                    if hasattr(result_with_pose, "hypothesis"):
                        # Extract the ObjectHypothesis from ObjectHypothesisWithPose
                        hyp = ObjectHypothesis()
                        hyp.class_id = result_with_pose.hypothesis.class_id
                        hyp.score = result_with_pose.hypothesis.score
                        camera_detection_msg.results.append(hyp)
            else:
                camera_detection_msg.results = []
                self.get_logger().warn(
                    "Detection3D has no results field or empty results"
                )

            # Extract 3D position from Detection3D
            # Detection3D can have position in two places:
            # 1. results[0].pose.pose.position - the detected object's 3D position in meters
            # 2. bbox.center.position - the 3D bounding box center position in meters.
            #
            # For depthai-ros-driver with spatial detection, the results pose position
            # is usually more reliable as it represents the actual detected object position.
            # The bbox center might contain pixel coordinates if not properly configured.

            position_source = None
            position_candidate = None

            # Try results pose position first (usually more reliable for spatial detections)
            if (
                hasattr(detection, "results")
                and detection.results
                and hasattr(detection.results[0], "pose")
                and hasattr(detection.results[0].pose, "pose")
            ):
                pos = detection.results[0].pose.pose.position
                position_candidate = pos
                position_source = "results[0].pose.pose.position"

                # Validate: check if it looks like a valid 3D position or pixel coords
                # Pixel coordinates typically: x,y in 0-2000 range, z=0 or very small
                is_pixel_coords = pos.z <= 0.01 or (
                    abs(pos.x) < 3000 and abs(pos.y) < 3000 and pos.z < 0.1
                )
                if self.enable_logging and is_pixel_coords:
                    self.get_logger().warn(
                        f"Results pose position looks like pixels: "
                        f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                    )

            # Fallback to bbox center if results position not available or invalid
            if (
                position_candidate is None
                and hasattr(detection, "bbox")
                and detection.bbox
            ):
                pos = detection.bbox.center.position
                position_candidate = pos
                position_source = "bbox.center.position"
                # Validate: check if it looks like a valid 3D position or pixel coords
                is_pixel_coords = pos.z <= 0.01 or (
                    abs(pos.x) < 3000 and abs(pos.y) < 3000 and pos.z < 0.1
                )
                if self.enable_logging and is_pixel_coords:
                    self.get_logger().warn(
                        f"Bbox center position looks like pixels: "
                        f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                    )

            # Set the position
            if position_candidate is not None:
                camera_detection_msg.position = position_candidate
                if self.enable_logging:
                    self.get_logger().info(
                        f"Position from {position_source}: "
                        f"({position_candidate.x:.2f},"
                        f" {position_candidate.y:.2f},"
                        f" {position_candidate.z:.2f})"
                    )
            else:
                # Default to origin if no valid position available
                from geometry_msgs.msg import Point

                camera_detection_msg.position = Point()
                # Log warning with details about what we found
                bbox_pos = (
                    detection.bbox.center.position
                    if hasattr(detection, "bbox") and detection.bbox
                    else None
                )
                results_pos = (
                    detection.results[0].pose.pose.position
                    if (
                        hasattr(detection, "results")
                        and detection.results
                        and hasattr(detection.results[0], "pose")
                    )
                    else None
                )
                self.get_logger().warn(
                    f"No valid 3D position found in Detection3D. "
                    f"Bbox center: {bbox_pos}, Results pose: {results_pos}. "
                    f"Values may be pixel coordinates (check depthai-ros-driver configuration)."
                )

            return camera_detection_msg

        except Exception as e:
            self.get_logger().error(
                f"Error converting detection message: {e}"
            )
            return None

    def publish_detections(self, detection_array_msg):
        """
        Process incoming Detection3DArray and publish individual CameraDetections.

        Args: detection_array_msg: Detection3DArray message from depthai-ros-driver

        """
        try:
            # Validate message structure
            if not hasattr(detection_array_msg, "detections"):
                self.get_logger().warn("Detection3DArray has no detections field")
                return

            # Validate detections list
            if not isinstance(detection_array_msg.detections, (list, tuple)):
                self.get_logger().warn(
                    "detections field is not a list/tuple, skipping message"
                )
                return

            # Convert and publish each detection
            detection_count = 0
            for detection in detection_array_msg.detections:
                camera_detection_msg = self.convert_message(detection)
                if camera_detection_msg is not None:
                    self.publisher_.publish(camera_detection_msg)
                    detection_count += 1

                    if self.enable_logging:
                        self.get_logger().info(
                            f"Published camera detection: "
                            f"position=({camera_detection_msg.position.x:.2f}, "
                            f"{camera_detection_msg.position.y:.2f}, "
                            f"{camera_detection_msg.position.z:.2f}), "
                            f"classes={len(camera_detection_msg.results)}"
                        )

            if detection_count > 0 and not self.enable_logging:
                # Only log summary if detailed logging is disabled
                self.get_logger().debug(
                    f"Processed {detection_count} detection(s) from camera"
                )

        except Exception as e:
            self.get_logger().error(
                f"Error processing Detection3DArray: {e}"
            )


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
