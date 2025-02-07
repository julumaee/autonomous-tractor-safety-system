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
            'radar_simulator_UART = simulations.radar_simulator_UART:main',
            'radar_simulator_can = simulations.radar_simulator_can:main',
            'agopen_simulator = simulations.agopen_simulator:main',
            'target_to_fuse_simulator = simulations.target_to_fuse_simulator:main',
            'object_simulator = simulations.simulate_objects:main',
        ],
    },
)
