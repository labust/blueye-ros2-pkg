import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value=os.path.expanduser('~/rov_logs'),
        description='Directory where CSV log files are written',
    )
    log_rate_arg = DeclareLaunchArgument(
        'log_rate',
        default_value='1.0',
        description='CSV write rate [Hz]',
    )

    logger_node = Node(
        name='rov_data_logger',
        package='blueye_ros2_interface',
        executable='rov_data_logger',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'log_dir':  LaunchConfiguration('log_dir'),
            'log_rate': LaunchConfiguration('log_rate'),
        }],
        remappings=[
            ('fix',         '/fix'),
            ('heading_deg', '/heading_deg'),
            ('depth',             '/blueye/depth'),
            ('imu',               '/blueye/imu'),
            ('battery_state',     '/blueye/battery_state'),
            ('altitude',          '/blueye/altitude'),
            ('water_temperature', '/blueye/water_temperature'),
            ('locator_position_relative_wrt_topside', '/uwgps/locator_position_relative_wrt_topside'),
            ('locator_position_topside_ned',           '/uwgps/locator_position_topside_ned'),
            ('locator_position_global',                '/uwgps/locator_position_global'),
        ],
    )

    return LaunchDescription([log_dir_arg, log_rate_arg, logger_node])