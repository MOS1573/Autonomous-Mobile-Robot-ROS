"""Start everything the robot needs to drive and sense.

    ros2 launch amr_bringup bringup.launch.py

Starts the robot model, the micro-ROS agent for the Teensy, the RPLIDAR A1 and
the EKF that fuses wheel odometry with the IMU. Resulting interface:
    /cmd_vel  ->  Teensy  ->  /odom/unfiltered, /imu/data
    EKF       ->  /odometry/filtered, tf odom -> base_footprint
    RPLIDAR   ->  /scan
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teensy_port = LaunchConfiguration("teensy_port")
    lidar_port = LaunchConfiguration("lidar_port")

    return LaunchDescription([
        DeclareLaunchArgument("teensy_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("amr_description"), "launch", "description.launch.py"]))),

        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=["serial", "--dev", teensy_port],
            output="screen",
        ),

        # RPLIDAR A1: 360 deg, 12 m range, ~5.5 Hz
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar",
            parameters=[{
                "serial_port": lidar_port,
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "angle_compensate": True,
            }],
            output="screen",
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[PathJoinSubstitution([FindPackageShare("amr_bringup"), "config", "ekf.yaml"])],
            output="screen",
        ),
    ])
