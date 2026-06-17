import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    blueye_pkg = get_package_share_directory('blueye_ros2_interface')
    uwgps_pkg  = get_package_share_directory('uwgpsg2_ros2_interface')

    # launch arguments 
    launch_blueye = DeclareLaunchArgument(
        'launch_blueye',
        default_value='true',
        description='Launch the Blueye telemetry node',
    )
    launch_uwgps = DeclareLaunchArgument(
        'launch_uwgps',
        default_value='true',
        description='Launch the WaterLinked UWGPS G2 ROS bridge node',
    )

    # Blueye telemetry node 
    blueye_node = Node(
        namespace='blueye',
        name='blueye_telemetry',
        package='blueye_ros2_interface',
        executable='blueye_telemetry',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(blueye_pkg, 'config', 'blueye_telemetry_params.yaml')],
        condition=IfCondition(LaunchConfiguration('launch_blueye')),
    )

    # WaterLinked UWGPS G2 localization node
    uwgps_node = Node(
        namespace='uwgps',
        name='waterlinked_localization_node',
        package='uwgpsg2_ros2_interface',
        executable='uwgpsg2_ros2_interface',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(uwgps_pkg, 'config', 'waterlinked_node_params.yaml')],
        remappings=[
            ('fix',                                    'fix'),
            ('navrelposned',                           'navrelposned'),
            ('locator_position_relative_wrt_topside',  'locator_position_relative_wrt_topside'),
            ('locator_position_global',                'locator_position_global'),
            ('locator_position_topside_ned',           'locator_position_topside_ned'),
            ('locator_acoustic_diagnostics',           'locator_acoustic_diagnostics'),
        ],
        condition=IfCondition(LaunchConfiguration('launch_uwgps')),
    )

    # Data logger node 
    logger_node = Node(
        name='rov_data_logger',
        package='blueye_ros2_interface',
        executable='rov_data_logger',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(blueye_pkg, 'config', 'rov_data_logger_params.yaml')],
        remappings=[
            # Blueye topics
            ('imu_calibrated',                 '/blueye/imu_calibrated'),
            ('imu1',                           '/blueye/imu1'),
            ('imu2',                           '/blueye/imu2'),
            ('attitude',                       '/blueye/attitude'),
            ('depth',                          '/blueye/depth'),
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
            # WaterLinked UWGPS topics
            ('locator_position_relative_wrt_topside', '/uwgps/locator_position_relative_wrt_topside'),
            ('locator_position_topside_ned',          '/uwgps/locator_position_topside_ned'),
            ('locator_position_global',               '/uwgps/locator_position_global'),
        ],
    )

    return LaunchDescription([
        launch_blueye,
        launch_uwgps,
        blueye_node,
        uwgps_node,
        logger_node,
    ])
