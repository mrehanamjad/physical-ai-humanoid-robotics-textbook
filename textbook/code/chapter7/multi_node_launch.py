from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<your_ros2_package_name>', # Replace with your package name, e.g., 'my_robot_bringup'
            executable='publisher_node',
            name='my_publisher',
            output='screen'
        ),
        Node(
            package='<your_ros2_package_name>', # Replace with your package name, e.g., 'my_robot_bringup'
            executable='subscriber_node',
            name='my_subscriber',
            output='screen'
        )
    ])
