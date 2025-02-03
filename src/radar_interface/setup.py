from setuptools import find_packages, setup

package_name = 'radar_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'serial'],
    zip_safe=True,
    maintainer='eemil',
    maintainer_email='eemil.kulmala@hotmail.com',
    description='A package for interfacing the radar sensor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_node_UART = radar_interface.radar_node_UART:main',
            'radar_node_can = radar_interface.radar_node_can:main',
        ],
    },
)
