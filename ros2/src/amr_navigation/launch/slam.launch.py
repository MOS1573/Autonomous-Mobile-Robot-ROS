"""Build a map with slam_toolbox while driving the robot by hand.

    ros2 launch amr_navigation slam.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard      # hold Shift to strafe
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
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
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("bringup", default_value="true",
                              description="Start the robot drivers too (false when they already run)"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("amr_bringup"), "launch", "bringup.launch.py"])),
            condition=IfCondition(LaunchConfiguration("bringup")),
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[PathJoinSubstitution([pkg, "config", "slam.yaml"])],
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([pkg, "config", "navigation.rviz"])],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
