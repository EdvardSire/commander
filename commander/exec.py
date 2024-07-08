#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion, Point


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

    values = [(1.0, 1.0, 0.0, 1.0), (2.0, 2.0, 0.0, 1.0), ( 3.0, 3.0, 0.0, 1.0,)]  # fmt: skip
    goal_poses = [
        PoseStamped(
            header=Header(frame_id="map"),
            pose=Pose(
                position=Point(x=x, y=y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=z, w=w),
            ),
        )
        for x, y, z, w in values
    ]

    _ = navigator.getPath(initial_pose, goal_poses[0])

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            navigator.info(
                f"Executing current waypoint: {feedback.current_waypoint}/{len(goal_poses)}"
            )
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some follow waypoints request change to demo preemption
            if now - nav_start > Duration(seconds=35.0):
                goal_pose4 = PoseStamped()
                goal_pose4.header.frame_id = "map"
                goal_pose4.header.stamp = now.to_msg()
                goal_pose4.pose.position.x = 0.0
                goal_pose4.pose.position.y = 0.0
                goal_pose4.pose.orientation.w = 1.0
                goal_pose4.pose.orientation.z = 0.0
                goal_poses = [goal_pose4]
                nav_start = now
                navigator.followWaypoints(goal_poses)
    navigator.get_logger().info("Waypoint mission FINISHED!")

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == "__main__":
    main()
