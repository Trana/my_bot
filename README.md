## My first robot

## Commands

# Source
source /home/mikaeltrana/Documents/robot/install/local_setup.bash 

# Build Ros
colcon build --symlink-install

# Generate new xacro file for sim mode
xacro /home/mikaeltrana/Documents/robot/my_bot/src/diff_drive_robot/description/models/urdf/robot.urdf.xacro>/home/mikaeltrana/Documents/robot/my_bot/src/diff_drive_robot/description/models/urdf/robot.urdf sim_mode:=true


# Exanple using transform
ros2 run tf2_ros static_transform_publisher 2 1 0 0.785 0 0 world robot_1
ros2 run tf2_ros static_transform_publisher 1 0 0 0 0 0 robot_1 robot_2
ros2 run rviz2 rviz2

# Run joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Start Rviz from config
rviz2 -d src/config/view_bot.rviz

source ./local_setup.bash 
ros2 launch ros_gz_example_bringup diff_drive.launch.py

# Run lidar driver node
ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -p frame_id:=lidar_link -p angel_compensate:=true -p scan_mode:=Standard -p serial_baudrate:=115200

# Installl Ros2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ign-ros2-control

# Run keyboard controller for Ros2 controll
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel_unstamped

# List ssh servers on local network
sudo nmap -p 22 192.168.86.0/24

# Run slam
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/mikaeltrana/Documents/robot/my_bot/src/diff_drive_robot/bringup/config/mapper_params_online_async.yaml use_sim_time:=true

# Use map saved from using slam and use it with map server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=test_map_v1.yaml -p use_sim_time:=true
ros2 run nav2_util lifecycle_bringup map_server

# Use AMCL localization with map from map server 
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
ros2 run nav2_util lifecycle_bringup amcl