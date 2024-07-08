import rclpy
from rclpy.action.server import ActionServer
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action._follow_waypoints import (
    FollowWaypoints_Goal,
    FollowWaypoints_Feedback,
    FollowWaypoints_Result,
)
from time import sleep


class FollowWaypointsServer(Node):
    def __init__(self):
        super().__init__("follow_waypoints_server")
        self._action_server = ActionServer(
            self, FollowWaypoints, "follow_waypoints", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("STARTING Waypoint mission")
        incoming_msg: FollowWaypoints_Goal = goal_handle.request
        number_of_waypoints_to_go = len(incoming_msg.poses)

        feedback = FollowWaypoints_Feedback()
        while number_of_waypoints_to_go > 0:
            feedback.current_waypoint = number_of_waypoints_to_go
            goal_handle.publish_feedback(feedback)
            sleep(5)
            number_of_waypoints_to_go -= 1

        goal_handle.succeed()
        return FollowWaypoints_Result()


def main(args=None):
    rclpy.init(args=args)
    server = FollowWaypointsServer()
    rclpy.spin(server)


if __name__ == "__main__":
    main()
