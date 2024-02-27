## My first robot

## Commands

# Build Ros
colcon build --symlink-install

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