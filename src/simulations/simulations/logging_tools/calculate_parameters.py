# Copyright 2024 Eemil Kulmala, University of Oulu
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

import numpy as np
import rclpy
from rclpy.node import Node


class TransformationCalculator(Node):

    def __init__(self):
        super().__init__('transformation_calculator')
        self.get_logger().info('Calculating transformation from camera to radar...')

        # Predefined points
        camera_raw = np.array([
            [0.1824275404214859, -0.839985728263855, 4.863906383514404],
            [0.19886426627635956, -0.571864128112793, 3.844054937362671],
            [0.13760696351528168, -0.3450128436088562, 2.8372786045074463],
            [0.07274454832077026, -0.08879309147596359, 1.906651258468628],
            [-0.5041002035140991, -0.8767547011375427, 4.965237617492676],
            [-1.467220425605774, -0.7923550009727478, 4.766627788543701]
        ])

        # Re-map to radar coordinate frame: [z, -x, -y]
        camera_points = np.array([
            [pt[2], -pt[0], -pt[1]] for pt in camera_raw
        ])

        radar_points = np.array([
            [5.1000000000000085, -0.04999999999999716, 0.0],
            [4.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [2.0, 0.0, 0.20000000000000284],
            [5.0, -0.9499999999999957, 0.0],
            [4.8500000000000085, -1.9000000000000000, -0.3000000000000000]
        ])

        R, T = self.compute_kabsch(camera_points, radar_points)

        self.get_logger().info('Rotation matrix R:\n' + str(R))
        self.get_logger().info('Translation vector T:\n' + str(T))

    def compute_kabsch(self, cam_pts, radar_pts):
        centroid_cam = np.mean(cam_pts, axis=0)
        centroid_rad = np.mean(radar_pts, axis=0)

        cam_centered = cam_pts - centroid_cam
        rad_centered = radar_pts - centroid_rad

        H = cam_centered.T @ rad_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        T = centroid_rad - R @ centroid_cam
        return R, T


def main(args=None):
    rclpy.init(args=args)
    node = TransformationCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
