#!/usr/bin/env bash
# Lanza el driver del RPLIDAR A1M8 (Slamtec) en la Raspi -> /scan (frame "laser", ~7 Hz, modo Express).
# Requiere: sudo apt install ros-jazzy-rplidar-ros  y la regla udev (/dev/jonas_lidar).
# Ojo: rplidar.launch.py del paquete tiene el puerto fijo en /dev/ttyUSB0 (que es el U2D2), por eso se usa ros2 run.
set -e
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}
source /opt/ros/jazzy/setup.bash
[ -e /dev/jonas_lidar ] || { echo "No existe /dev/jonas_lidar: ¿lidar conectado? ¿regla udev instalada?" >&2; exit 1; }
exec ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/jonas_lidar -p serial_baudrate:=115200 \
    -p frame_id:=laser -p angle_compensate:=true "$@"
