"""Navigate autonomously on a saved map with Nav2 (AMCL + planner + controller).

    ros2 launch amr_navigation navigation.launch.py map:=$HOME/maps/my_map.yaml

In RViz: "2D Pose Estimate" to set the start pose, then "Nav2 Goal".
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("amr_navigation")

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Full path to the map yaml file"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("bringup", default_value="true",
                              description="Start the robot drivers too (false when they already run)"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("amr_bringup"), "launch", "bringup.launch.py"])),
            condition=IfCondition(LaunchConfiguration("bringup")),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": PathJoinSubstitution([pkg, "config", "nav2.yaml"]),
                "use_sim_time": "false",
            }.items(),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "config", "navigation.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
