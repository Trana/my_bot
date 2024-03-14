# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('bringup')
    pkg_project_gazebo = get_package_share_directory('gazebo')
    pkg_project_description = get_package_share_directory('description')
    

    ## Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_project_bringup,'launch','robot_state_publisher.launch.py')]), 
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    twist_mux_params = os.path.join(pkg_project_bringup,'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_drive_base_controller/cmd_vel_unstamped')]
    )

    ## Camera
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_project_bringup,'launch','camera.launch.py')])
    )

    ## Lidar
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_project_bringup,'launch','rplidar.launch.py')])
    )
    
    ## RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(pkg_project_bringup, 'config','my_controller.yaml')

    ## Ros2 Control manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params_file]
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    ## Ros2 Control
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["diff_drive_base_controller"]
    )

    delayed_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_controller_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    ## Launch
    return LaunchDescription([
        # DeclareLaunchArgument('rviz', default_value='true',
        #                       description='Open RViz.'),
        robot_state_publisher,
        twist_mux,
        # rviz,
        delayed_controller_manager,
        delayed_diff_drive_controller_spawner,
        delayed_joint_broad_spawner,
        camera,
        lidar
    ])
