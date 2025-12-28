from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='simple_node',
            name='included_simple_node',
            output='screen'
        )
    ])
