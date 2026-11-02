"""
Launch diffdrive_bot simulation with obstacle detector and drive controller,
the rotate action server, and the square driver action client.

Usage:
    ros2 launch my_robot_nodes square.launch.py
    ros2 launch my_robot_nodes square.launch.py side_length:=0.8
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    diffdrive_pkg_share = get_package_share_directory("diffdrive_bot")

    side_length = ParameterValue(LaunchConfiguration("side_length"), value_type=float)
    speed = LaunchConfiguration("speed")
    stop_distance = LaunchConfiguration("stop_distance")

    # Include the robot's control launch file (starts Gazebo, bridge, RViz, obstacle_detector, drive_controller, drive_distance_server)
    control_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diffdrive_pkg_share, "launch", "control.launch.py")
        ),
        launch_arguments={
            "speed": speed,
            "stop_distance": stop_distance,
        }.items(),
    )

    # Turn away node (starts disabled so it displays on the graph without interfering with square driver)
    turn_away_node = Node(
        package="my_robot_nodes",
        executable="turn_away",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Rotate action server
    rotate_server_node = Node(
        package="my_robot_nodes",
        executable="rotate_server",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Square driver action client
    square_driver_node = Node(
        package="my_robot_nodes",
        executable="square_driver",
        parameters=[{
            "side_length": side_length,
            "use_sim_time": True,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "side_length",
            default_value="0.5",
            description="Side length of the square trajectory in meters",
        ),
        DeclareLaunchArgument(
            "speed",
            default_value="0.3",
            description="Forward speed in m/s",
        ),
        DeclareLaunchArgument(
            "stop_distance",
            default_value="0.5",
            description="Stop distance in meters",
        ),
        control_simulation,
        turn_away_node,
        rotate_server_node,
        square_driver_node,
    ])
