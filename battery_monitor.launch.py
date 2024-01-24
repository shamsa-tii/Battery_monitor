from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='battery_monitor',
            namespace='battery_monitor',
            executable='battery_monitor_node',
            name='battery_monitor_node'
        ),
        Node(
            package='battery_monitor',
            namespace='battery_monitor',
            executable='drone_control_service',
            name='drone_control_service'
        ),
        Node(
            package='battery_monitor',
            namespace='battery_monitor',
            executable='drone_plugin',
            name='drone_plugin'
        ),
    ])
