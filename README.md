# rover-2024
Software for CRATER 2024 Univerity Rover Challenge

# Odrive Remote Module
cd ~/rover-2024/ros2_ws/
colcon build --packages-select odrive_remote
source ~/rover-2024/ros2_ws/install/setup.bash
ros2 run odrive_remote odrive_remote_node
