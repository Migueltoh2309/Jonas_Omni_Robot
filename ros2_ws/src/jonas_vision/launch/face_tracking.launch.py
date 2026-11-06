"""Cara + seguimiento de caras:  ros2 launch jonas_vision face_tracking.launch.py [debug:=true] [mirror:=false]"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("mirror", default_value="true"),
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        Node(package="jonas_face", executable="face", output="screen"),
        Node(package="jonas_vision", executable="face_tracker", output="screen",
             parameters=[{"debug": LaunchConfiguration("debug"),
                          "mirror": LaunchConfiguration("mirror"),
                          "device": LaunchConfiguration("device")}]),
    ])
