# Copyright 2026 Eemil Kulmala, University of Oulu
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    test_name = LaunchConfiguration("test_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "test_name",
                default_value="test_1",
                description="Name used for output CSV filenames.",
            ),
            Node(
                package="simulations",
                executable="real_world_logger",
                name="real_world_logger_node",
                output="screen",
                parameters=[
                    {
                        "test_name": test_name,
                        "use_sim_time": use_sim_time,  # Use real time for real-world logging
                    }
                ],
            ),
        ]
    )
