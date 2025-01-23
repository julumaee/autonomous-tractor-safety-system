from setuptools import find_packages, setup

package_name = 'simulation_scripts'

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
            "camera_simulation = simulation_scripts.camera_simulator:main",
            "radar_simulator = simulation_scripts.radar_simulator_UART:main",
            "agopen_simulator = simulation_scripts.agopen_simulator:main",
            "target_simulator = simulation_scripts.target_to_fuse_simulator:main",
        ],
    },
)
