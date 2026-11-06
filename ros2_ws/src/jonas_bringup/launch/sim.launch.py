"""Jonas simulado (laptop / Docker): robot + lidar + persona simulados, modo auto, app web, cara y RViz.

    ros2 launch jonas_bringup sim.launch.py [rviz:=true] [face:=true] [web_port:=8080]

Abre la app en http://localhost:8080, pestaña Auto → INICIAR, y mira en RViz cómo se mueve.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory("jonas_bringup"), "rviz", "jonas.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("face", default_value="true", description="ventana con los ojos"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        Node(package="jonas_auto", executable="sim_node", output="screen"),
        Node(package="jonas_auto", executable="auto", output="screen"),
        Node(package="jonas_web", executable="server", output="screen",
             arguments=["--port", LaunchConfiguration("web_port")]),
        Node(package="jonas_face", executable="face", output="screen", parameters=[{"fullscreen": False}],
             condition=IfCondition(LaunchConfiguration("face"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
