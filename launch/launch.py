from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="commander",
                executable="exec",
                name="exec",
            ),
            Node(
                package="commander",
                executable="follow_waypoints",
                name="follow_waypoints",
            ),
        ]
    )
