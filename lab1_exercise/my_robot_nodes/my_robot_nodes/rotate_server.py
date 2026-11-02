#!/usr/bin/env python3
"""Action server node that rotates diffdrive_bot in place to a specified angle.

Implements Lab 1 Exercise Step 6:
- Serves the /rotate action (my_robot_interfaces/action/Rotate).
- Tracks rotation angle using odometry yaw accumulation to handle [-pi, pi] wrap-around.
- Uses a MultiThreadedExecutor so odometry continues to update during goal execution.
- Publishes feedback, handles cancellation, and rejects invalid goals (zero angle).
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Calculate the yaw (rotation about the z-axis in radians) from an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """Compute (target - current) wrapped to [-pi, pi] to handle angle boundaries."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):
    """Action server providing the /rotate action for in-place rotations."""

    def __init__(self):
        super().__init__("rotate_server")

        # Declare parameters
        self.declare_parameter("turn_speed", 0.5)

        self.current_yaw = None

        # Reentrant callback group allows odometry callbacks to run while the goal executes
        callback_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.on_odom,
            10,
            callback_group=callback_group,
        )

        self.action_server = ActionServer(
            self,
            Rotate,
            "rotate",
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=callback_group,
        )

        self.get_logger().info("rotate_server initialized and ready")

    def on_odom(self, msg):
        """Update current robot orientation from /odom."""
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal_request):
        """Validate incoming goal request."""
        # Reject goal if angle is exactly zero or odometry has not yet been received
        if math.isclose(goal_request.angle, 0.0, abs_tol=1e-5) or self.current_yaw is None:
            self.get_logger().warn(
                f"Rejecting rotate goal: angle={goal_request.angle}, odom_received={self.current_yaw is not None}"
            )
            return GoalResponse.REJECT

        self.get_logger().info(f"Accepted rotate goal: {goal_request.angle:.3f} rad")
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        """Accept goal cancellation requests."""
        self.get_logger().info("Received goal cancellation request")
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        """Execute the rotation by commanding angular velocity and accumulating odometry yaw deltas."""
        target_angle = goal_handle.request.angle
        turn_speed = abs(self.get_parameter("turn_speed").value)

        # Positive angle turns left (+z), negative angle turns right (-z)
        turn_direction = 1.0 if target_angle > 0.0 else -1.0

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = turn_direction * turn_speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        last_yaw = self.current_yaw
        accumulated_turned = 0.0

        self.get_logger().info(
            f"Starting rotation: target={target_angle:.3f} rad, speed={turn_speed:.2f} rad/s"
        )

        # Control loop at ~20 Hz
        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                goal_handle.canceled()
                result.turned = float(accumulated_turned)
                self.get_logger().info(f"Rotation canceled after {accumulated_turned:.3f} rad")
                return result

            # Accumulate small incremental changes in yaw
            if self.current_yaw is not None and last_yaw is not None:
                delta_yaw = angle_difference(self.current_yaw, last_yaw)
                accumulated_turned += delta_yaw
                last_yaw = self.current_yaw

            feedback.turned = float(accumulated_turned)
            goal_handle.publish_feedback(feedback)

            # Check if target angle has been achieved
            if abs(accumulated_turned) >= abs(target_angle):
                break

            # Send movement command
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        # Stop robot upon completing goal
        self.cmd_vel_pub.publish(Twist())
        goal_handle.succeed()
        result.turned = float(accumulated_turned)
        self.get_logger().info(f"Rotation completed: turned {result.turned:.3f} rad")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RotateServer()
    try:
        # Spin with MultiThreadedExecutor to ensure odom callback runs concurrently with execute()
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
