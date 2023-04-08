
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(package='pkg', executable='node', output='screen'),
        Node(package='pkg', executable='fake_gps_node', name='fake_gps_node', output='screen'),
    ])