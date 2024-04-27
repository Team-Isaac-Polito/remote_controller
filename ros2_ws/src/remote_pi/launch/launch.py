from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share = get_package_share_directory("remote_pi")

    return LaunchDescription([
        Node(
            package='remote_pi',
            executable='remote',
            name='remote',
        ),
        Node(
            package='rviz2',
            name='rviz2',
            executable='rviz2',
            arguments=[f'-d {share}/config/rviz-config.rviz']
        )
    ])
