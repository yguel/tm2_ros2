import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro
from launch import LaunchContext, LaunchDescription

# Prerequisites: joint-state-publisher-gui
# Terminal: [key-in] shell cmd $ sudo apt install ros-humble-joint-state-publisher
#                              $ sudo apt install ros-humble-joint-state-publisher-gui
# Usage: Spawn a Techman robot model in the Rviz2.
# Example: Take TM12S Techman robot model as the default, so set 'tm12s.urdf.xacro' in robot_description_config
# Terminal:
#   * [key-in] shell cmd STANDARD USAGE:
#           $ ros2 launch tm_description view_robot.launch.py
#   * [key-in] shell cmd save produced URDF file:
#           $ ros2 launch tm_description view_robot.launch.py output_urdf_path:=/tmp/robot.urdf


known_robot_types = [
    "tm5s",
    "tm5sx",
    "tm7s",
    "tm7sx",
    "tm12s",
    "tm12sx",
    "tm14s",
    "tm14sx",
    "tm25s",
    "tm30s",
]


def process_urdf_and_launch_nodes(context: LaunchContext):
    # 1. Safely resolve LaunchConfigurations into Python strings at runtime
    robot_type = context.perform_substitution(LaunchConfiguration("robot_type")).lower()
    output_path = context.perform_substitution(LaunchConfiguration("output_urdf_path"))

    if robot_type not in known_robot_types:
        raise ValueError(
            f"Unknown robot type: {robot_type}. Known types: {', '.join(known_robot_types)}"
        )

    # 2. Build paths using standard strings
    description_path = "tm_description"
    xacro_file = f"{robot_type}.urdf.xacro"

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory(description_path),
            "xacro",
            xacro_file,
        )
    )
    urdf_content = robot_description_config.toxml()

    # 3. Save the URDF if an output path was provided
    if output_path:
        with open(output_path, "w") as f:
            f.write(urdf_content)

    # 4. Setup the nodes that depend on the generated URDF string
    robot_description = {"robot_description": urdf_content}
    rviz_config_file = os.path.join(
        get_package_share_directory(description_path), "rviz", "view_robot.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Return the configured nodes to the launch execution
    return [rviz_node, robot_state_publisher]


def generate_launch_description():
    # 1. Declare the launch argument
    output_urdf_path_arg = DeclareLaunchArgument(
        "output_urdf_path",
        default_value="",
        description="Path to save the generated URDF file",
    )

    robot_type_arg = DeclareLaunchArgument(
        "robot_type",
        default_value="tm12s",
        description="Type of the robot to be spawned, possible values are "
        + ", ".join(known_robot_types),
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    # Interface to command joint states by publishing them through a GUI
    joint_state_slider = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    return LaunchDescription(
        [
            output_urdf_path_arg,
            robot_type_arg,
            static_tf,
            joint_state_slider,
            OpaqueFunction(
                function=process_urdf_and_launch_nodes
            ),  # Executes Python logic and returns the dynamic nodes
        ]
    )
