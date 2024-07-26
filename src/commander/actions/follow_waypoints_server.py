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
from nav_msgs.msg import Path
from rclpy.action.server import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from actions.config import (
    OWN_CURRENT_WAYPOINT_TOPIC,
    OWN_NEXT_WAYPOINT_TOPIC,
    OWN_PATH_TOPIC,
    OWN_POSITION_TOPIC,
)


class FollowWaypointsServer(Node):
    def __init__(self):
        super().__init__("follow_waypoints_server")
        self.boat_length = 3
        self.waypoint_tolerance = 2 * self.boat_length
        self.position = Twist()

        self._action_server = ActionServer(
            self, FollowWaypoints, "follow_waypoints", self.execute_callback
        )
        self.position_subscriber = self.create_subscription(
            Twist, OWN_POSITION_TOPIC, self.position_callback, 10
        )
        self.los_current_pub = self.create_publisher(
            Point, OWN_CURRENT_WAYPOINT_TOPIC, 10
        )
        self.los_next_pub = self.create_publisher(Point, OWN_NEXT_WAYPOINT_TOPIC, 10)
        self.path_pub = self.create_publisher(Path, OWN_PATH_TOPIC, 10)

    def execute_callback(self, goal_handle):
        # self.get_logger().info("STARTING Waypoint mission")
        incoming_poses_stamped: List[PoseStamped] = goal_handle.request.poses
        waypoints = [
            (
                Point(
                    x=pose_stamped.pose.position.x,
                    y=pose_stamped.pose.position.y,
                    z=0.0,
                )
            )
            for pose_stamped in incoming_poses_stamped
        ]
        feedback = FollowWaypoints_Feedback()
        finished = False
        self.los_current_pub.publish(
            Point(x=self.position.linear.x, y=self.position.linear.y, z=0.0)
        )

        def control_vessel():
            index = 0
            self.los_next_pub.publish(waypoints[index])
            nonlocal finished
            while not finished:
                self.path_pub.publish(
                    Path(
                        header=Header(
                            stamp=self.get_clock().now().to_msg(), frame_id="map"
                        ),
                        poses=incoming_poses_stamped[index:],
                    )
                )
                distance = math.sqrt(
                    (waypoints[index].x - self.position.linear.x) ** 2
                    + (waypoints[index].y - self.position.linear.y) ** 2
                )
                self.get_logger().info(
                    f"Point {index+1}/{len(waypoints)}, distance: {distance}"
                )
                if distance <= self.waypoint_tolerance:
                    index += 1
                    self.los_current_pub.publish(
                        Point(x=self.position.linear.x, y=self.position.linear.y, z=0.0)
                    )
                    if not (index == len(waypoints)):
                        self.los_next_pub.publish(waypoints[index])
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
