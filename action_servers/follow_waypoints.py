import rclpy
from rclpy.action.server import ActionServer
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints


class FollowWaypointsServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            FollowWaypoints,
            'follow_waypoints',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        # result = Fibonacci.Result()
        pass


def main(args=None):
    rclpy.init(args=args)
    server = FollowWaypointsServer()
    rclpy.spin(server)

if __name__ == '__main__':
    main()
