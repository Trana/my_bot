import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('bringup')
        
    ## RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'robot.rviz')],
       
    )

    main_cam_image_transport_cmd = Node(
    package='image_transport',
    executable='republish',
    arguments=['compressed', 'raw'],
    remappings=[
        ('in/compressed', '/image_raw/compressed'),
        ('out', '/image_raw/uncompressed')]
)

    ## Launch
    return LaunchDescription([
        main_cam_image_transport_cmd,
        rviz,        
    ])

