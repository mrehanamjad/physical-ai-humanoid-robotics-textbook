from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the parameter file
    param_file = PathJoinSubstitution(
        [
            FindPackageShare('<your_ros2_package_name>'), # Replace with your package name
            'params', # Assuming a 'params' directory in your package
            'params.yaml'
        ]
    )

    return LaunchDescription([
        Node(
            package='<your_ros2_package_name>', # Replace with your package name
            executable='param_node',
            name='my_hello_world_node',
            output='screen',
            parameters=[param_file] # Load parameters from the YAML file
        )
    ])
