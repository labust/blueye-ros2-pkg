from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import TextSubstitution
from launch.actions import DeclareLaunchArgument

import os

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = LaunchDescription()
    
    blueye_params = os.path.join(
        get_package_share_directory('blueye_ros2_interface'),
        'config',
        'ros2_interface_node_params.yaml'
        )
    
    
    blueye_node =  Node(
            namespace='blueye',
            name='ros2_interface_node',
            package='blueye_ros2_interface',            
            executable='blueye_interface',            
            output='screen',
            emulate_tty=True,
            parameters= [blueye_params],
            remappings=[]
        )
        
    ld.add_action(blueye_node)
    
    
    
    return ld

