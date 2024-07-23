#!/usr/bin/env python3
import math
import threading
from time import sleep
from typing import List

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.action._follow_waypoints import (
    FollowWaypoints_Feedback,
    FollowWaypoints_Result,
)
from rclpy.action.server import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FollowWaypointsServer(Node):
    def __init__(self):
        super().__init__("follow_waypoints_server")
        self._action_server = ActionServer(
            self, FollowWaypoints, "follow_waypoints", self.execute_callback
        )
        self.position = Twist()
        self.position_subscriber = self.create_subscription(
            Twist, "/revolt/sim/stc/position/hull", self.position_callback, 10
        )
        self.boat_length = 3
        self.radius_of_acceptance = 2 * self.boat_length

        self.los_current_pub = self.create_publisher(
            Point, "/revolt/control/los_current_waypoint", 10
        )
        self.los_next_pub = self.create_publisher(
            Point, "/revolt/control/los_next_waypoint", 10
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("STARTING Waypoint mission")
        incoming_poses_stamped: List[PoseStamped] = goal_handle.request.poses
        waypoints = [
            (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in incoming_poses_stamped
        ]
        self.get_logger().info(f"Number of poses: {len(waypoints)}")
        feedback = FollowWaypoints_Feedback()
        finished = False
        self.los_current_pub.publish(
            Point(x=self.position.linear.x, y=self.position.linear.y, z=0.0)
        )

        def control_vessel():
            index = 0
            self.los_next_pub.publish(
                Point(x=waypoints[index][0], y=waypoints[index][1], z=0.0)
            )
            nonlocal finished
            while not finished:
                distance = math.sqrt(
                    (waypoints[index][0] - self.position.linear.x) ** 2
                    + (waypoints[index][1] - self.position.linear.y) ** 2
                )
                self.get_logger().info(
                    f"Point {index+1}/{len(waypoints)}, distance: {distance}"
                )
                if distance <= self.radius_of_acceptance:
                    index += 1
                    self.los_current_pub.publish(
                        Point(x=self.position.linear.x, y=self.position.linear.y, z=0.0)
                    )
                    if not (index == len(waypoints)):
                        self.los_next_pub.publish(
                            Point(x=waypoints[index][0], y=waypoints[index][1], z=0.0)
                        )
                    else:
                        finished = True

                feedback.current_waypoint = index
                goal_handle.publish_feedback(feedback)
                sleep(0.1)

        threading.Thread(target=control_vessel).start()

        while not finished:
            sleep(1)

        goal_handle.succeed()
        return FollowWaypoints_Result()

    def position_callback(self, msg):
        self.position = msg


def main(args=None):
    rclpy.init(args=args)
    server = FollowWaypointsServer()
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    executor.spin()


if __name__ == "__main__":
    main()
