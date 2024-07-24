#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Point
from nav_msgs.msg import Path

import numpy as np


def get_simple_values():
    return [(1.0, 1.0, 0.0, 1.0), (2.0, 2.0, 0.0, 1.0), (3.0, 2.0, 0.0, 1.0)]


def get_square_values():
    return [(-20.0, -70.0, 0.0, 1.0), (-10.0, 70.0, 0.0, 1.0), (-70.0, 70.0, 0.0, 1.0), (-70.0, -70.0, 0.0, 1.0) ]  # fmt: skip


def get_figure_8():
    num_points = 30
    scale_factor = 50

    t = np.linspace(0, 2 * np.pi, num_points)

    x = np.sin(t)
    y = np.sin(t) * np.cos(t)
    x *= scale_factor
    y *= scale_factor

    if plot := False:
        import matplotlib.pyplot as plt

        plt.plot(x, y)
        plt.scatter(x, y, color="r")
        plt.grid(True)
        plt.show()

    global_offset = 0
    return [(x - global_offset, y + global_offset, 0.0, 1.0) for x, y in zip(x, y)]


def get_collision_path():
    return [(100.0, 0.0, 0.0, 1.0)]


def main():
    rclpy.init()
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x, initial_pose.pose.position.y = 0.0, 0.0
    initial_pose.pose.orientation.z, initial_pose.pose.orientation.w = 0.0, 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    navigator.lifecycleStartup()
    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    goal_poses = [
        PoseStamped(
            header=Header(stamp=navigator.get_clock().now().to_msg(), frame_id="map"),
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=z, w=w),
            ),
        )
        for x, y, z, w in get_collision_path()
    ]

    path: Path | None = navigator.getPath(initial_pose, goal_poses[-1])

    navigator.followWaypoints(path.poses)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        # if feedback and i % 5 == 0:
        #     navigator.info(
        #         f"Executing current waypoint: {feedback.current_waypoint}/{len(goal_poses)}"
        #     )

        # if navigator.get_clock().now() - nav_start > Duration(seconds=600.0):
        #     navigator.cancelTask()

    navigator.get_logger().info("Waypoint mission FINISHED!")
    navigator.lifecycleShutdown()
    exit(0)

    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print("Goal succeeded!")
    # elif result == TaskResult.CANCELED:
    #     print("Goal was canceled!")
    # elif result == TaskResult.FAILED:
    #     print("Goal failed!")
    # else:
    #     print("Goal has an invalid return status!")


if __name__ == "__main__":
    main()
