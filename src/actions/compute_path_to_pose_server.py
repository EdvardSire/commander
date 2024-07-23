import rclpy
from rclpy.action.server import ActionServer
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action._compute_path_to_pose import ComputePathToPose_Result


class ComputePathToPoseServer(Node):
    def __init__(self):
        super().__init__("follow_waypoints_server")
        self._action_server = ActionServer(
            self, ComputePathToPose, "compute_path_to_pose", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info("Path to Pose valid")
        goal_handle.succeed()
        return ComputePathToPose_Result()


def main(args=None):
    rclpy.init(args=args)
    server = ComputePathToPoseServer()
    rclpy.spin(server)


if __name__ == "__main__":
    main()
