from setuptools import find_packages, setup

package_name = 'sensor_fusion'

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
    maintainer_email='eemil.kulmala@oulu.fi',
    description='A package for performing target-level sensor fusion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = sensor_fusion.fusion_node:main'
        ],
    },
)
