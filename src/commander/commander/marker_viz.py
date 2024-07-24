#!/usr/bin/env python3

import rclpy
from actions.config import OTHER_POSITIONS_TOPIC, OWN_POSITION_TOPIC, OWN_VIZ_TOPIC
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion, Twist, Vector3
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


class MarkerViz(Node):
    def __init__(self):
        super().__init__("marker_viz")
        self.position = Twist()
        self.pose_array = PoseArray()

        self.position_subscriber = self.create_subscription(
            Twist, OWN_POSITION_TOPIC, self.position_callback, 10
        )
        self.other_position_subscriber = self.create_subscription(
            PoseArray, OTHER_POSITIONS_TOPIC, self.other_position_callback, 10
        )

        self.marker_pub = self.create_publisher(MarkerArray, OWN_VIZ_TOPIC, 10)

    def position_callback(self, msg):
        self.position = msg
        markers = MarkerArray()
        markers.markers.append(  # pyright: ignore
            Marker(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"),
                id=0,
                type=Marker.CUBE,
                pose=Pose(
                    position=Point(x=msg.linear.x, y=msg.linear.y, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=msg.angular.z, w=1.0),
                ),
                scale=Vector3(x=3.0, y=1.0, z=1.0),
                color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
            )
        )

        try:
            if not self.pose_array.poses:
                self.get_logger().warn("self.pose_array.poses is empty.")
                return

            for index, pose in enumerate(self.pose_array.poses):
                marker = Marker(
                    header=Header(
                        stamp=self.get_clock().now().to_msg(), frame_id="map"
                    ),
                    id=index + 1,
                    type=Marker.CUBE,
                    pose=Pose(
                        position=Point(
                            x=pose.position.x,
                            y=pose.position.y,
                            z=0.0,
                        ),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    scale=Vector3(x=3.0, y=1.0, z=1.0),
                    color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                )
                markers.markers.append(marker)

        except Exception as e:
            self.get_logger().error(f"{e}")

        self.marker_pub.publish(markers)

    def other_position_callback(self, msg):
        self.pose_array = msg


def main(args=None):
    rclpy.init(args=args)
    node = MarkerViz()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
