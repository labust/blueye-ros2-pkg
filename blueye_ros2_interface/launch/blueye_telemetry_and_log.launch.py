import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('blueye_ros2_interface')

    # Blueye telemetry node
    telemetry_node = Node(
        namespace='blueye',
        name='blueye_telemetry',
        package='blueye_ros2_interface',
        executable='blueye_telemetry',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(pkg, 'config', 'blueye_telemetry_params.yaml')],
    )

    # Data logger node
    logger_node = Node(
        name='rov_data_logger',
        package='blueye_ros2_interface',
        executable='rov_data_logger',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(pkg, 'config', 'rov_data_logger_params.yaml')],
        remappings=[
            ('depth',             '/blueye/depth'),
            ('imu',               '/blueye/imu'),
            ('battery_state',     '/blueye/battery_state'),
            ('altitude',          '/blueye/altitude'),
            ('water_temperature', '/blueye/water_temperature'),
        ],
    )

    return LaunchDescription([telemetry_node, logger_node])