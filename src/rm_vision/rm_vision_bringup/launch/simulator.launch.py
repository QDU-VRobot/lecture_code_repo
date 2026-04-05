import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch")
)


def generate_launch_description():
    from common import (
        launch_params,
        robot_state_publisher,
        node_params,
        get_tracker_node,
        get_trajectory_node,
    )

    robot_type = LaunchConfiguration("robot")

    detector_node = Node(
        package="armor_detector",
        executable="armor_detector_node",
        emulate_tty=True,
        output="both",
        parameters=[node_params, {"robot_type": robot_type}],
        arguments=[
            "--ros-args",
            "--log-level",
            "armor_detector:=" + launch_params["detector_log_level"],
        ],
        on_exit=Shutdown(),
    )

    simulator_driver_node = Node(
        package="rm_simulator_driver",
        executable="rm_simulator_driver_node",
        name="simulator_driver",
        output="both",
        emulate_tty=True,
        parameters=[{"robot_type": robot_type}],
        ros_arguments=[
            "--ros-args",
            "--log-level",
            ["simulator_driver:=", launch_params["simulator_log_level"]],
        ],
        on_exit=Shutdown(),
    )

    delay_simulator_node = TimerAction(
        period=1.0,
        actions=[simulator_driver_node],
    )

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[get_tracker_node(robot_type)],
    )

    delay_trajectory_node = TimerAction(
        period=2.5,
        actions=[get_trajectory_node(robot_type)],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="default"),
            robot_state_publisher,
            detector_node,
            delay_simulator_node,
            delay_tracker_node,
            delay_trajectory_node,
        ]
    )
