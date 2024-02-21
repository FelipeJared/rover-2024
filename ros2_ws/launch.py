import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from ament_index_python.packages import get_package_share_directory


from launch_ros.actions import Node

def generate_launch_description():
    config_filepath = LaunchConfiguration('config_filepath')
    
    return LaunchDescription([
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            'xbox', TextSubstitution(text='.config.yaml')]),
            
        # Node(
        #     package='teleop_twist_joy',
        #     executable='launch/teleop-launch.py',
        #     parameters=[{
        #         'joy_config': 'xbox',
        #     }]
        # ),
        Node(
            package='joy', 
            executable='joy_node', 
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        Node(
            package='teleop', executable='teleop_node',
            name='teleop_node'),
        # Node(
        #     package='teleop_twist_joy', executable='teleop_node',
        #     name='teleop_twist_joy_node', parameters=[config_filepath]),
        # Node(
        #     package='usb_cam', 
        #     executable='usb_cam_node_exe', 
        #     name='usb_cam',
        #     parameters=[{
        #         'camera_name': 'logitech',
        #         'framerate': 30.0,
        #     }]),
        # Node(
        #     package='dynamixel_sdk_examples',
        #     executable='read_write_node',
        # )
        Node(
            package='rqt_gui', executable='rqt_gui',
            name='rqt_gui', arguments=['--perspective-file', 'driving.perspective']),
    ])