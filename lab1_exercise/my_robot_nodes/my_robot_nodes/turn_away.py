#!/usr/bin/env python3
"""ROS 2 node that avoids obstacles by steering away when an obstacle is detected too close.

Implements Lab 1 Exercise Steps 2, 3, 4, and 5:
- Subscribes to /obstacle (lab_interfaces/msg/ObstacleInfo).
- Publishes to /cmd_vel (geometry_msgs/msg/Twist).
- Publishes to /robot_status (my_robot_interfaces/msg/RobotStatus).
- Provides services /turn_away_enable (std_srvs/srv/SetBool) and /set_turn_speed (my_robot_interfaces/srv/SetTurnSpeed).
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):
    """Avoid obstacles by driving forward when clear and turning away when too close."""

    def __init__(self):
        super().__init__("turn_away")

        # Exercise Step 3 & Step 5: Declare parameters
        self.declare_parameter("forward_speed", 0.2)
        self.declare_parameter("turn_speed", 0.5)
        self.declare_parameter("max_turn_speed", 1.5)

        # Exercise Step 5: The node starts disabled
        self.enabled = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", 10)

        # Subscriber
        self.create_subscription(ObstacleInfo, "obstacle", self.on_obstacle, 10)

        # Services
        self.create_service(SetBool, "turn_away_enable", self.on_enable)
        self.create_service(SetTurnSpeed, "set_turn_speed", self.on_set_turn_speed)

        self.get_logger().info("turn_away node initialized (starts disabled)")

    def on_enable(self, request, response):
        """Service callback to enable or disable obstacle avoidance."""
        if request.data:
            self.enabled = True
            response.success = True
            response.message = "turn_away enabled"
        else:
            self.stop()
            response.success = True
            response.message = "turn_away disabled"

        self.get_logger().info(response.message)
        return response

    def on_set_turn_speed(self, request, response):
        """Service callback to validate and dynamically update the turning speed."""
        max_turn_speed = self.get_parameter("max_turn_speed").value

        # Reject speed of zero, negative speed, or speed exceeding max_turn_speed
        if request.turn_speed <= 0.0 or request.turn_speed > max_turn_speed:
            response.success = False
            response.message = (
                f"Rejected: turn speed must be > 0 and <= {max_turn_speed:.2f} rad/s "
                f"(requested: {request.turn_speed:.2f} rad/s)"
            )
        else:
            self.set_parameters([Parameter("turn_speed", value=float(request.turn_speed))])
            response.success = True
            response.message = f"turn_speed set to {request.turn_speed:.2f} rad/s"

        self.get_logger().info(response.message)
        return response

    def on_obstacle(self, msg):
        """Callback triggered on each ObstacleInfo message from the obstacle detector."""
        # Exercise Step 4: Prepare RobotStatus message
        status_msg = RobotStatus()
        status_msg.obstacle_distance = float(msg.distance)

        # If disabled, maintain "stopped" state and do not publish movement
        if not self.enabled:
            status_msg.state = "stopped"
            self.status_pub.publish(status_msg)
            return

        # Exercise Step 3: Read parameters on each message so changes take effect immediately
        forward_speed = self.get_parameter("forward_speed").value
        turn_speed = self.get_parameter("turn_speed").value

        twist = Twist()

        if not msg.too_close:
            # Path is clear: drive forward, no angular turn
            status_msg.state = "forward"
            twist.linear.x = float(forward_speed)
            twist.angular.z = 0.0
        else:
            # Obstacle too close: stop forward motion and turn on the spot
            status_msg.state = "turning"
            twist.linear.x = 0.0

            # Direction logic: positive angle means obstacle is to the left -> turn right (-z)
            if msg.angle > 0.0:
                twist.angular.z = -float(turn_speed)
            else:
                twist.angular.z = float(turn_speed)

        # Publish movement command and status
        self.cmd_vel_pub.publish(twist)
        self.status_pub.publish(status_msg)

    def stop(self):
        """Stop robot movement and disable the controller."""
        self.enabled = False
        # Publish exactly one zero-velocity command when switching off
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TurnAway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
