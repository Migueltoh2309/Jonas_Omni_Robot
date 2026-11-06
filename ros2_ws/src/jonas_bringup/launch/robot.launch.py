"""Jonas real (Raspi): lidar, cara + seguimiento de caras, modo auto y app web.

    ros2 launch jonas_bringup robot.launch.py [lidar:=true] [vision:=true] [face:=true]

El micro-ROS Agent (ruedas) se sigue lanzando aparte con raspi/run_agent.sh, porque antes reinicia el ESP32.
Pendiente: jonas_base (odometría) y jonas_arms cuando estén listos.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    L = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("vision", default_value="true"),
        DeclareLaunchArgument("face", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/jonas_lidar"),
        DeclareLaunchArgument("lidar_yaw", default_value="0.0", description="rad, calibrar en el robot"),
        Node(package="rplidar_ros", executable="rplidar_composition", output="screen",
             parameters=[{"serial_port": L("lidar_port"), "serial_baudrate": 115200, "frame_id": "laser",
                          "angle_compensate": True}],
             condition=IfCondition(L("lidar"))),
        Node(package="tf2_ros", executable="static_transform_publisher", output="log",
             arguments=["--z", "0.25", "--frame-id", "base_link", "--child-frame-id", "laser"]),
        Node(package="jonas_face", executable="face", output="screen", condition=IfCondition(L("face"))),
        Node(package="jonas_vision", executable="face_tracker", output="screen", condition=IfCondition(L("vision"))),
        Node(package="jonas_auto", executable="auto", output="screen", parameters=[{"lidar_yaw": L("lidar_yaw")}]),
        Node(package="jonas_web", executable="server", output="screen"),
    ])
