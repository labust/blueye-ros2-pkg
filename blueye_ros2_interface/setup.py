from setuptools import setup
import os
from glob import glob


package_name = 'blueye_ros2_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.perspective'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matko Batos',
    maintainer_email='matko.batos@fer.unizg.hr',
    description='ROS2 interface for the Blueye Pioneer ROV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blueye_interface  = blueye_ros2_interface.blueye_interface:main',
            'blueye_telemetry  = blueye_ros2_interface.blueye_telemetry:main',
            'rov_data_logger   = blueye_ros2_interface.rov_data_logger:main',
        ],
    },
)
