from numpy import append
import rclpy
from rclpy.action.server import ActionServer
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action._compute_path_to_pose import (
    ComputePathToPose_Result,
    ComputePathToPose_Goal
)
from std_msgs.msg import Header


from actions.informed_rrt_star import InformedRRTStar


class ComputePathToPoseServer(Node):
    def __init__(self):
        super().__init__("compute_path_to_pose_server")
        self.expand_dis = 3
        self.goal_sample_rate = 10
        self.max_iter = 100
        self.border_offset = 20
        self.action_server = ActionServer(
            self, ComputePathToPose, "compute_path_to_pose", self.execute_callback
        )

    def execute_callback(self, goal_handle):
        request: ComputePathToPose_Goal = goal_handle.request
        start_pose = (request.start.pose.position.x, request.start.pose.position.y)
        goal_pose = (request.goal.pose.position.x, request.goal.pose.position.y)
        """"
                                                         top_right
       |----------------------------------------------|------*
       |                                              |      |
       |    start pose                            goal|pose  |
       |    --*->                                     *------| border_offset
       |                                                     |
       *-----------------------------------------------------|
       bottom_left
        """
        bottom_left = (start_pose[0]-self.border_offset, start_pose[1]-self.border_offset)
        top_right = (goal_pose[0]+self.border_offset, goal_pose[1]+self.border_offset)
        # TODO HANDLE ALL CASES OF PADDING
        self.informed_rrt_star = InformedRRTStar(
            start=[request.start.pose.position.x, request.start.pose.position.y],
            goal=[request.goal.pose.position.x, request.goal.pose.position.y],
            obstacle_list=[],
            rand_area=[bottom_left[0], bottom_left[1], top_right[0], top_right[1]],
            expand_dis=self.expand_dis,
            goal_sample_rate=self.goal_sample_rate,
            max_iter=self.max_iter
        )
        rrt_path, time_elapsed = self.informed_rrt_star.informed_rrt_star_search(animation=False)
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
                path.poses.append(
                    PoseStamped(
                        header=Header(),
                        pose=Pose(
                            position=Point(x=x, y=y, z=0.0),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                        )
                    )
                )

            return ComputePathToPose_Result(
                        path=path,
                        planning_time=Duration(sec=int(time_elapsed))
                    )


def main(args=None):
    rclpy.init(args=args)
    server = ComputePathToPoseServer()
    rclpy.spin(server)


if __name__ == "__main__":
    main()
