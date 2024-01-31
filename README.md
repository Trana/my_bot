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