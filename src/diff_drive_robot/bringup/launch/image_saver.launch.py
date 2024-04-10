import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('bringup')
    image_saver_params = os.path.join(pkg_project_bringup,'config','image_saver.yaml')

    return LaunchDescription([
        Node(
            package='image_view',
            executable='image_saver',
            output='screen',
            remappings=[
            ('image', 'image_raw/uncompressed')],    
            parameters=[image_saver_params],        
        )
    ])
