from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arrow_marker_publisher',
            executable='LA_arrow',
        ),
        Node(
            package='arrow_marker_publisher',
            executable='AV_arrow',
        ),
        Node(
            package='arrow_marker_publisher',
            executable='AV_arrow_curve',
        )
    ])