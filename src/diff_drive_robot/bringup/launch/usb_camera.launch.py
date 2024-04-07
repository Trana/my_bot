import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bringup_package_name='bringup'    
    usb_camera_params_file = os.path.join(get_package_share_directory(bringup_package_name),'config','usb_camera.yaml')

    return LaunchDescription([

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen',
            # namespace='camera',§
            parameters=[usb_camera_params_file]
        )
    ])   