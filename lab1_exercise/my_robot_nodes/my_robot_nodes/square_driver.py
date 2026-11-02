#!/usr/bin/env python3
"""Action client node that drives diffdrive_bot in a square pattern.

Implements Lab 1 Exercise Step 7:
- Sequential client to:
  1. /drive_distance (lab_interfaces/action/DriveDistance)
  2. /rotate (my_robot_interfaces/action/Rotate)
- Drives four sides with side_length parameter (default 0.5 m) and 90-degree left turns.
- Halts and logs if any goal fails or is rejected.
"""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):
    """Coordinates driving in a four-sided square using DriveDistance and Rotate actions."""

    def __init__(self):
        super().__init__("square_driver")

        # Declare side_length parameter (default: 0.5 m)
        self.declare_parameter("side_length", 0.5)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Reentrant callback group for action client operations
        callback_group = ReentrantCallbackGroup()

        # Action clients
        self.drive_client = ActionClient(
            self,
            DriveDistance,
            "drive_distance",
            callback_group=callback_group,
        )
        self.rotate_client = ActionClient(
            self,
            Rotate,
            "rotate",
            callback_group=callback_group,
        )

        # Start sequence in a dedicated background worker thread so the ROS executor can spin
        self.worker_thread = threading.Thread(target=self.run_sequence, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("square_driver node started")

    def on_drive_feedback(self, feedback_msg):
        """Log drive_distance progress."""
        self.get_logger().info(f"[Drive] Travelled {feedback_msg.feedback.travelled:.2f} m")

    def on_rotate_feedback(self, feedback_msg):
        """Log rotate progress."""
        self.get_logger().info(f"[Rotate] Turned {feedback_msg.feedback.turned:.2f} rad")

    def stop_robot(self):
        """Publish zero-velocity command to stop robot safely."""
        self.cmd_vel_pub.publish(Twist())

    def wait_for_future(self, future, timeout_sec=60.0):
        """Wait for a ROS future to complete without blocking the ROS executor thread."""
        start_time = time.time()
        while not future.done() and rclpy.ok():
            if time.time() - start_time > timeout_sec:
                return False
            time.sleep(0.05)
        return future.done()

    def run_sequence(self):
        """Execute the square-driving sequence over 4 sides."""
        # Wait a short moment for system launch to stabilize
        time.sleep(1.0)

        self.get_logger().info("Waiting for /drive_distance and /rotate action servers...")
        if not self.drive_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Action server /drive_distance not available!")
            return
        if not self.rotate_client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("Action server /rotate not available!")
            return

        side_length = float(self.get_parameter("side_length").value)
        self.get_logger().info(f"Starting square driving routine with side_length={side_length:.2f} m")

        # 4 sides in a square
        for side in range(1, 5):
            if not rclpy.ok():
                break

            self.get_logger().info(f"--- Side {side}/4: Driving forward {side_length:.2f} m ---")

            # 1. Drive forward one side length
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = side_length

            send_future = self.drive_client.send_goal_async(
                drive_goal,
                feedback_callback=self.on_drive_feedback,
            )
            if not self.wait_for_future(send_future):
                self.get_logger().error(f"Timeout waiting for drive goal acceptance on side {side}")
                self.stop_robot()
                return

            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f"DriveDistance goal rejected on side {side}")
                self.stop_robot()
                return

            result_future = goal_handle.get_result_async()
            if not self.wait_for_future(result_future):
                self.get_logger().error(f"Timeout waiting for drive result on side {side}")
                self.stop_robot()
                return

            drive_result_wrapper = result_future.result()
            if (
                not drive_result_wrapper
                or drive_result_wrapper.status != GoalStatus.STATUS_SUCCEEDED
                or not drive_result_wrapper.result.reached
            ):
                status_code = drive_result_wrapper.status if drive_result_wrapper else "Unknown"
                self.get_logger().error(
                    f"DriveDistance failed on side {side} (status={status_code})"
                )
                self.stop_robot()
                return

            self.get_logger().info(
                f"Side {side} drive complete: travelled {drive_result_wrapper.result.travelled:.2f} m"
            )

            # 2. Rotate a quarter turn left (pi / 2 radians)
            turn_angle = math.pi / 2.0  # ~1.5708 rad
            self.get_logger().info(f"--- Side {side}/4: Rotating left {turn_angle:.3f} rad ---")

            rotate_goal = Rotate.Goal()
            rotate_goal.angle = float(turn_angle)

            rot_send_future = self.rotate_client.send_goal_async(
                rotate_goal,
                feedback_callback=self.on_rotate_feedback,
            )
            if not self.wait_for_future(rot_send_future):
                self.get_logger().error(f"Timeout waiting for rotate goal acceptance on side {side}")
                self.stop_robot()
                return

            rot_handle = rot_send_future.result()
            if not rot_handle or not rot_handle.accepted:
                self.get_logger().error(f"Rotate goal rejected on side {side}")
                self.stop_robot()
                return

            rot_result_future = rot_handle.get_result_async()
            if not self.wait_for_future(rot_result_future):
                self.get_logger().error(f"Timeout waiting for rotate result on side {side}")
                self.stop_robot()
                return

            rot_result_wrapper = rot_result_future.result()
            if (
                not rot_result_wrapper
                or rot_result_wrapper.status != GoalStatus.STATUS_SUCCEEDED
            ):
                status_code = rot_result_wrapper.status if rot_result_wrapper else "Unknown"
                self.get_logger().error(
                    f"Rotate failed on side {side} (status={status_code})"
                )
                self.stop_robot()
                return

            self.get_logger().info(
                f"Side {side} rotate complete: turned {rot_result_wrapper.result.turned:.3f} rad"
            )

        self.get_logger().info("=== Successfully completed all 4 sides of the square! ===")
        self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
