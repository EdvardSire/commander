#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose, PoseArray
import math

from std_msgs.msg import Header

from actions.config import (
    OTHER_POSITIONS_TOPIC,
    OWN_NEXT_WAYPOINT_TOPIC,
    OWN_POSITION_TOPIC,
)


class MockBoat(Node):
    def __init__(self):
        super().__init__("mock_boat")
        self.position = Twist()
        self.velocity = Twist()
        self.current_waypoint = None
        self.boat_length = 3
        self.waypoint_tolerance = 2 * self.boat_length
        self.speed_multiply = 5.0

        self.position_publisher = self.create_publisher(Twist, OWN_POSITION_TOPIC, 10)
        self.other_position_publisher = self.create_publisher(
            PoseArray, OTHER_POSITIONS_TOPIC, 10
        )
        self.waypoint_subscriber = self.create_subscription(
            Point, OWN_NEXT_WAYPOINT_TOPIC, self.waypoint_callback, 10
        )

        self.timer = self.create_timer(0.1, self.update_position)

    def waypoint_callback(self, msg):
        self.current_waypoint = msg

    def update_position(self):
        self.update_other_position()
        dt = 0.1  # Time step

        self.position.linear.x += self.velocity.linear.x * dt
        self.position.linear.y += self.velocity.linear.y * dt
        self.position.linear.z += self.velocity.linear.z * dt
        self.position_publisher.publish(self.position)

        if self.current_waypoint:
            distance = math.sqrt(
                (self.current_waypoint.x - self.position.linear.x) ** 2
                + (self.current_waypoint.y - self.position.linear.y) ** 2
            )

            if distance < self.waypoint_tolerance:
                self.velocity.linear.x = 0.0
                self.velocity.linear.y = 0.0
                self.velocity.linear.z = 0.0
                self.current_waypoint = None
                return

            # Calculate the desired velocity to move towards the waypoint
            angle_to_waypoint = math.atan2(
                self.current_waypoint.y - self.position.linear.y,
                self.current_waypoint.x - self.position.linear.x,
            )
            self.velocity.linear.x = self.speed_multiply * math.cos(angle_to_waypoint)
            self.velocity.linear.y = self.speed_multiply * math.sin(angle_to_waypoint)

    def update_other_position(self):
        self.other_position_publisher.publish(
            PoseArray(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"),
                poses=[
                    Pose(position=Point(x=50.0)),
                    # Pose(position=Point(x=50.0, y=5.0))
                ],
            )
        )


def main(args=None):
    rclpy.init(args=args)
    mock_boat = MockBoat()
    rclpy.spin(mock_boat)
    # rclpy.spin(mock_boat, executor=MultiThreadedExecutor())
    mock_boat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
