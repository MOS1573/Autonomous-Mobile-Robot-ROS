"""Publish the robot model (URDF) and its static transforms.

    ros2 launch amr_description description.launch.py rviz:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("amr_description")
    urdf = PathJoinSubstitution([pkg, "urdf", "amr.urdf.xacro"])
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "model.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
