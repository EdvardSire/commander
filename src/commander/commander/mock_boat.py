import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import math

class MockBoat(Node):
    def __init__(self):
        super().__init__('mock_boat')
        self.position = Twist()
        self.velocity = Twist()
        self.current_waypoint = None
        self.waypoint = Point
        self.waypoint_tolerance = 0.5  # Distance tolerance to consider the waypoint reached

        self.position_publisher = self.create_publisher(Twist, '/revolt/sim/stc/position/hull', 10)
        self.waypoint_subscriber = self.create_subscription(Point, '/revolt/control/los_next_waypoint', self.waypoint_callback, 10)

        self.timer = self.create_timer(0.01, self.update_position)


    def waypoint_callback(self, msg):
        self.waypoint = msg

    def update_position(self):
        # self.get_logger().info(str(self.position))
        dt = 0.1  # Time step

        self.position.linear.x += self.velocity.linear.x * dt
        self.position.linear.y += self.velocity.linear.y * dt
        self.position.linear.z += self.velocity.linear.z * dt

        self.position_publisher.publish(self.position)

        if self.waypoints:
            self.current_waypoint = self.waypoints[0]

            distance = math.sqrt(
                (self.current_waypoint.x - self.position.linear.x) ** 2 +
                (self.current_waypoint.y - self.position.linear.y) ** 2
            )

            if distance < self.waypoint_tolerance:
                self.waypoints.pop(0)  # Remove the reached waypoint
                if not self.waypoints:
                    self.current_waypoint = None
                    self.velocity.linear.x = 0.0
                    self.velocity.linear.y = 0.0
                    self.velocity.linear.z = 0.0
                    return

            angle_to_waypoint = math.atan2(
                self.current_waypoint.y - self.position.linear.y,
                self.current_waypoint.x - self.position.linear.x
            )
            self.velocity.linear.x = (speed_multiply := 5) * math.cos(angle_to_waypoint)
            self.velocity.linear.y = speed_multiply * math.sin(angle_to_waypoint)

def main(args=None):
    rclpy.init(args=args)
    mock_boat = MockBoat()
    rclpy.spin(mock_boat)
    mock_boat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

