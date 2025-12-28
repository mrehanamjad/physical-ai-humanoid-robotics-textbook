from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the sub_launch.py file
    # Replace '<your_ros2_package_name>' with your actual package name
    sub_launch_file_path = os.path.join(
        get_package_share_directory('<your_ros2_package_name>'),
        'launch', # Assuming sub_launch.py is in a 'launch' directory within your package
        'sub_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sub_launch_file_path)
        ),
        # You can add other nodes or actions here
        # Node(
        #     package='<your_ros2_package_name>',
        #     executable='another_node',
        #     name='my_another_node',
        #     output='screen'
        # )
    ])
