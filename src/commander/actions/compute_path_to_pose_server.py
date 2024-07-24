import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action._compute_path_to_pose import (
    ComputePathToPose_Goal,
    ComputePathToPose_Result,
)
from nav_msgs.msg import Path
from rclpy.action.server import ActionServer
from rclpy.node import Node
from std_msgs.msg import Header

from actions.config import OTHER_POSITIONS_TOPIC

from actions.informed_rrt_star import InformedRRTStar, create_square


class ComputePathToPoseServer(Node):
    def __init__(self):
        super().__init__("compute_path_to_pose_server")
        self.expand_dis = 3
        self.goal_sample_rate = 10
        self.max_iter = 400
        self.border_offset = 20
        self.other_positions = PoseArray()

        self.action_server = ActionServer(
            self, ComputePathToPose, "compute_path_to_pose", self.execute_callback
        )
        self.other_position_sub = self.create_subscription(
            PoseArray, OTHER_POSITIONS_TOPIC, self.other_position_callback, 10
        )

    def execute_callback(self, goal_handle):
        request: ComputePathToPose_Goal = goal_handle.request
        start_pose = (request.start.pose.position.x, request.start.pose.position.y)
        goal_pose = (request.goal.pose.position.x, request.goal.pose.position.y)
        """"
                                                          (max_x, max_y)
       |----------------------------------------------|------*
       |                                              |      |
       |    start pose                            goal|pose  |
       |    --*->                                     *------| border_offset
       |                                                     |
       *-----------------------------------------------------|
       (min_x, min_y)
        """
        min_x = min(start_pose[0], goal_pose[0]) - self.border_offset
        max_x = max(start_pose[0], goal_pose[0]) + self.border_offset
        min_y = min(start_pose[1], goal_pose[1]) - self.border_offset
        max_y = max(start_pose[1], goal_pose[1]) + self.border_offset

        for pose in self.other_positions.poses:
            assert (pose.position.x != 0) or (pose.position.y != 0)

        self.informed_rrt_star = InformedRRTStar(
            start=[request.start.pose.position.x, request.start.pose.position.y],
            goal=[request.goal.pose.position.x, request.goal.pose.position.y],
            obstacle_list=[
                create_square(
                    pose.position.x,
                    pose.position.y,
                    10,
                )
                for pose in self.other_positions.poses
            ],
            rand_area=[min_x, min_y, max_x, max_y],
            expand_dis=self.expand_dis,
            goal_sample_rate=self.goal_sample_rate,
            max_iter=self.max_iter,
        )
        rrt_path, time_elapsed = self.informed_rrt_star.informed_rrt_star_search(
            animation=False
        )
        if rrt_path is None:
            self.get_logger().fatal("Can't compute path")
            goal_handle.abort()
            return
        else:
            path = Path()
            self.get_logger().info(f"{rrt_path}")
            goal_handle.succeed()

            rrt_path.reverse()
            for x, y in rrt_path:
                path.poses.append(  # pyright: ignore
                    PoseStamped(
                        header=Header(),
                        pose=Pose(
                            position=Point(x=x, y=y, z=0.0),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                        ),
                    )
                )

            return ComputePathToPose_Result(
                path=path, planning_time=Duration(sec=int(time_elapsed))
            )

    def other_position_callback(self, msg):
        self.other_positions = msg


def main(args=None):
    rclpy.init(args=args)
    server = ComputePathToPoseServer()
    rclpy.spin(server)


if __name__ == "__main__":
    main()
