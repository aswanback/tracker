from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracker_pkg',
            executable='fake_gps_node',
            name='fake_gps_node'),
        Node(
            package='tracker_pkg',
            executable='fake_ui_node',
            name='fake_ui_node'),
        Node(
            package='tracker_pkg',
            executable='tracker_node',
            name='tracker_node'),
  ])