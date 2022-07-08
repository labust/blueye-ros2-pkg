import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()
      
    
    gstreamer_node_params = os.path.join(
        get_package_share_directory('blueye_ros2_interface'),
        'config',
        'gstreamer_topside_node_params.yaml'
        )
        
    gstreamer_remappings = [('image_raw', 'camera_img_raw_topside_stream'),
                  ('camera_info', 'camera_calibration_params_topside_stream')]
    
    gstreamer_node_topside = Node (
            namespace='blueye',
            name='gstreamer_node_topside',
            package='gscam',
            executable='gscam_node',
            output='screen',
            emulate_tty=True,
            parameters= [gstreamer_node_params],
            remappings=gstreamer_remappings,
            arguments=[]
    
        )
       
    ###################################################################
    
    pkg_dir = get_package_share_directory('blueye_ros2_interface')    
    launch_top_side = IncludeLaunchDescription( \
        PythonLaunchDescriptionSource( pkg_dir + \
        '/launch/start_blueye_interface_top_side.launch.py'))     
        
        
    ld.add_action(launch_top_side)
    ld.add_action(gstreamer_node_topside)    
    return ld

