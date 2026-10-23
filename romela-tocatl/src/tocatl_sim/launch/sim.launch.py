"""Launch the TOCATL MuJoCo sim: physics + gait + keyboard teleop.

    ros2 launch tocatl_sim sim.launch.py
    ros2 launch tocatl_sim sim.launch.py headless:=true teleop:=false
    ros2 launch tocatl_sim sim.launch.py rviz:=true

Simulation only -- nothing here touches CAN or motors.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tocatl_sim")
    default_params = os.path.join(share, "config", "sim.yaml")

    headless = DeclareLaunchArgument("headless", default_value="false")
    teleop = DeclareLaunchArgument(
        "teleop", default_value="true",
        description="start the keyboard teleop node (needs pynput; run "
                    "'ros2 run tocatl_sim teleop_keyboard' in a terminal otherwise)")
    rviz = DeclareLaunchArgument("rviz", default_value="false")
    params_file = DeclareLaunchArgument("params_file", default_value=default_params)

    sim_node = Node(
        package="tocatl_sim",
        executable="mujoco_sim_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file"),
                    {"headless": LaunchConfiguration("headless")}],
    )
    gait_node = Node(
        package="tocatl_sim",
        executable="gait_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )
    teleop_node = Node(
        package="tocatl_sim",
        executable="teleop_keyboard",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("teleop")),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        headless, teleop, rviz, params_file,
        sim_node, gait_node, teleop_node, rviz_node,
    ])
