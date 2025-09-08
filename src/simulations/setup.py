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

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'simulations'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config',
            [p for p in glob('config/*') if os.path.isfile(p)]),
        (f'share/{package_name}/launch',
            [p for p in glob('launch/*.launch.py') if os.path.isfile(p)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eemil',
    maintainer_email='eemil.kulmala@hotmail.com',
    description='Package inludes files needed for simulating the system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_simulator = simulations.camera_simulator:main',
            'radar_simulator_can = simulations.radar_simulator_can:main',
            'agopen_simulator = simulations.agopen_simulator:main',
            'target_to_fuse_simulator = simulations.target_to_fuse_simulator:main',
            'object_simulator = simulations.simulate_objects:main',
            'calibration_logger = simulations.calibration_logger:main',
            'calculate_parameters = simulations.calculate_parameters:main',
            'test_system_logger = simulations.test_system_logger:main'
        ],
    },
)
