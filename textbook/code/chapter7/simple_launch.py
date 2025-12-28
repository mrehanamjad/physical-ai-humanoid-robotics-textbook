from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chapter7_examples', # Assuming a package named chapter7_examples
            executable='simple_node',
            name='my_simple_node',
            output='screen'
        )
    ])
