#!/user/bin/env python3
from enum import Enum
import time
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import numpy as np
import rclpy
from shapely import LineString
from shapelysmooth import taubin_smooth, chaikin_smooth, catmull_rom_smooth
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import Header
from scipy.interpolate import splev, splprep

from rclpy.action.server import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import SmoothPath
from nav2_msgs.action._smooth_path import SmoothPath_Goal, SmoothPath_Result


def perpendicular_distance(point, line_start, line_end):
    """Calculate the perpendicular distance from a point to a line segment."""
    if np.array_equal(line_start, line_end):
        return np.linalg.norm(np.array(point) - np.array(line_start))
    else:
        line_vec = np.array(line_end) - np.array(line_start)
        point_vec = np.array(point) - np.array(line_start)
        line_len = np.linalg.norm(line_vec)
        line_unitvec = line_vec / line_len
        projection = np.dot(point_vec, line_unitvec)
        projection = np.clip(projection, 0, line_len)
        nearest = np.array(line_start) + projection * line_unitvec
        return np.linalg.norm(np.array(point) - nearest)


def douglas_peucker(points, epsilon):
    """Simplify the path using the Douglas-Peucker algorithm."""
    if len(points) < 2:
        return points

    dmax = 0
    index = 0
    for i in range(1, len(points) - 1):
        d = perpendicular_distance(points[i], points[0], points[-1])
        if d > dmax:
            dmax = d
            index = i

    if dmax > epsilon:
        rec_results1 = douglas_peucker(points[: index + 1], epsilon)
        rec_results2 = douglas_peucker(points[index:], epsilon)

        return rec_results1[:-1] + rec_results2
    else:
        return [points[0], points[-1]]


def basis_spline(geometry: LineString, smoothing_condition=20):
    waypoints = np.array([[x, y] for x, y in zip(*geometry.xy)])
    tck, _ = splprep([waypoints[:, 0], waypoints[:, 1]], s=smoothing_condition)
    u_fine = np.linspace(0, 1, len(waypoints) * 2)
    x_smooth, y_smooth = splev(u_fine, tck)
    return LineString(np.c_[x_smooth, y_smooth])


class SmootherType(Enum):
    TAUBIN = 0
    CHAIKIN = 1
    CATMULL_ROM = 2
    B_SPLINE = 3


class PathSmootherhServer(Node):
    def __init__(self):
        super().__init__("path_smoother_server")
        self.smoother_method = SmootherType.TAUBIN

        self.action_server = ActionServer(
            self, SmoothPath, "smooth_path", self.execute_callback
        )

        self.path_raw_publisher = self.create_publisher(Path, "/path_raw", 10)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        request: SmoothPath_Goal = goal_handle.request

        smoothed_path, time_elapsed = self.smooth(request.path)

        if smoothed_path is None:
            self.get_logger().error("Could not smooth path")
            return SmoothPath_Result()
        else:
            self.get_logger().info(f"Path smoothing took: {time_elapsed}")
            goal_handle.succeed()
            return SmoothPath_Result(path=smoothed_path)

    def smooth(self, path: Path):
        path.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="map")
        self.path_raw_publisher.publish(path)
        start_time = time.time()
        geometry = LineString(
            [
                (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
                for pose_stamped in path.poses
            ]
        )
        path_length_before = len(list(geometry.coords))
        geometry = self.post_process_path(geometry, self.smoother_method)
        self.get_logger().info(
            f"Path length was smoothed from: {path_length_before} to {len(list(geometry.coords))}"
        )

        smoothed_path = Path()
        for x, y in zip(*geometry.xy):
            smoothed_path.poses.append(  # pyrigth: ignore
                PoseStamped(
                    header=Header(),
                    pose=Pose(
                        position=Point(x=x, y=y, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                )
            )

        return smoothed_path, time.time() - start_time

    def post_process_path(
        self, geometry: LineString, method: SmootherType
    ) -> LineString:
        self.get_logger().info(f"Smoothed with: {method.name}")
        if method == SmootherType.TAUBIN:
            return taubin_smooth(geometry)

        elif method == SmootherType.CHAIKIN:
            geometry = LineString(
                douglas_peucker([(x, y) for x, y in zip(*geometry.xy)], epsilon=1.0)
            )
            return chaikin_smooth(geometry)

        elif method == SmootherType.CATMULL_ROM:
            geometry = LineString(
                douglas_peucker([(x, y) for x, y in zip(*geometry.xy)], epsilon=1.0)
            )
            return catmull_rom_smooth(geometry)

        elif method == SmootherType.B_SPLINE:
            return basis_spline(geometry, smoothing_condition=20)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathSmootherhServer())


if __name__ == "__main__":
    main()
