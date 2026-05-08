import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('blueye_ros2_interface')

    telemetry_node = Node(
        namespace='blueye',
        name='blueye_telemetry',
        package='blueye_ros2_interface',
        executable='blueye_telemetry',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(pkg, 'config', 'blueye_telemetry_params.yaml')],
    )

    logger_node = Node(
        name='rov_data_logger',
        package='blueye_ros2_interface',
        executable='rov_data_logger',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(pkg, 'config', 'rov_data_logger_params.yaml')],
        remappings=[
            ('depth',                          '/blueye/depth'),
            ('imu_calibrated',                 '/blueye/imu_calibrated'),
            ('imu1',                           '/blueye/imu1'),
            ('imu2',                           '/blueye/imu2'),
            ('attitude',                       '/blueye/attitude'),
            ('battery_state',                  '/blueye/battery_state'),
            ('water_temperature',              '/blueye/water_temperature'),
            ('canister_humidity',              '/blueye/canister_humidity'),
            ('canister_temperature',           '/blueye/canister_temperature'),
            ('control_force',                  '/blueye/control_force'),
            ('control_mode',                   '/blueye/control_mode'),
            ('controller_depth_error',         '/blueye/controller_depth_error'),
            ('controller_depth_health',        '/blueye/controller_depth_health'),
            ('controller_heading_error',       '/blueye/controller_heading_error'),
            ('controller_heading_health',      '/blueye/controller_heading_health'),
            ('magnetic_declination',           '/blueye/magnetic_declination'),
            ('cpu_temperature',                '/blueye/cpu_temperature'),
            ('dive_time',                      '/blueye/dive_time'),
        ],
    )

    return LaunchDescription([telemetry_node, logger_node])
