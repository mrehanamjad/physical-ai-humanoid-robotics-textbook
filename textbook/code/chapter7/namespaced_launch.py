from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robot1_nodes = [
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='publisher_node',
            name='my_publisher',
            namespace='robot1',
            output='screen'
        ),
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='subscriber_node',
            name='my_subscriber',
            namespace='robot1',
            output='screen'
        )
    ]

    robot2_nodes = [
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='publisher_node',
            name='my_publisher',
            namespace='robot2',
            output='screen'
        ),
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='subscriber_node',
            name='my_subscriber',
            namespace='robot2',
            output='screen'
        )
    ]

    return LaunchDescription(robot1_nodes + robot2_nodes)
