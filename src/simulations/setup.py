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

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "simulations"


def _collect_data_files(base_dir: str, package_name: str):
    """
    Collect files under base_dir preserving subdirectory structure.

    Returns a list of (install_dir, [files...]) tuples suitable for setuptools
    `data_files`.
    """
    base_dir = base_dir.rstrip(os.sep)
    install_map: dict[str, list[str]] = {}
    for root, _, files in os.walk(base_dir):
        if not files:
            continue
        rel_dir = os.path.relpath(root, ".")
        install_dir = os.path.join("share", package_name, rel_dir)
        for filename in files:
            src_path = os.path.join(root, filename)
            if os.path.isfile(src_path):
                install_map.setdefault(install_dir, []).append(src_path)
    return sorted(install_map.items())


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            f"share/{package_name}/config",
            [p for p in glob("config/*") if os.path.isfile(p)],
        ),
        (
            f"share/{package_name}/launch",
            [p for p in glob("launch/*.launch.py") if os.path.isfile(p)],
        ),
        (
            f"share/{package_name}/worlds",
            [p for p in glob("worlds/*.sdf") if os.path.isfile(p)],
        ),
    ]
    + _collect_data_files("models", package_name),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="eemil",
    maintainer_email="eemil.kulmala@hotmail.com",
    description="Package inludes files needed for simulating the system",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_simulator = simulations.system_integration.camera_simulator:main",
            "radar_simulator_can = simulations.system_integration.radar_simulator_can:main",
            "agopen_simulator = simulations.system_integration.agopen_simulator:main",
            "target_to_fuse = simulations.system_integration.target_to_fuse_simulator:main",
            "object_simulator = simulations.system_integration.simulate_objects:main",
            "twist_to_control = simulations.gazebo.twist_to_control:main",
            "control_to_twist = simulations.gazebo.control_to_twist:main",
            "lidar_to_radar = simulations.gazebo.lidar_to_radar:main",
            "camera_to_camera = simulations.gazebo.camera_to_camera:main",
            "spatial_from_yolo = simulations.gazebo.spatial_from_yolo:main",
            "laser_to_radar = simulations.gazebo.laser_to_radar:main",
            "visualize_tracks = simulations.gazebo.visualize_tracks:main",
            "ego_odom_sim = simulations.gazebo.ego_odom_sim:main",
            "detection_logger = simulations.logging_tools.detection_logger:main",
            "real_world_logger = simulations.logging_tools.real_world_logger:main",
            "gps_ego_motion = simulations.logging_tools.gps_ego_motion:main",
            "pedestrian_gt_bridge = simulations.gazebo.pedestrian_gt_bridge:main",
            "scripted_driver = simulations.gazebo.scripted_driver:main",
        ],
    },
)
