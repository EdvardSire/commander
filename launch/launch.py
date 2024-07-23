from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="commander",
            #     executable="mock_boat",
            #     name="mock_boat",
            # ),
            Node(
                package="commander",
                executable="exec",
                name="exec",
            ),
            Node(
                package="commander",
                executable="follow_waypoints_server",
                name="follow_waypoints_server",
            ),
            Node(
                package="commander",
                executable="compute_path_to_pose_server",
                name="compute_path_to_pose_server",
            ),
        ]
    )
