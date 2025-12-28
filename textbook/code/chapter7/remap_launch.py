from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='publisher_node',
            name='my_publisher',
            output='screen',
            remappings=[
                ('topic', 'remapped_topic') # Remap 'topic' to 'remapped_topic'
            ]
        ),
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='subscriber_node',
            name='my_subscriber',
            output='screen',
            remappings=[
                ('topic', 'remapped_topic') # Remap 'topic' to 'remapped_topic'
            ]
        )
    ])
