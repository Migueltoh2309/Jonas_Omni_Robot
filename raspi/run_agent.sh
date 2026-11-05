#!/usr/bin/env bash
# Lanza el micro-ROS Agent (ROS 2 Jazzy) para el ESP32 de Jonas, en la Raspberry Pi.
# Uso: raspi/run_agent.sh            (Ctrl+C para detener)
# Variables opcionales: PORT (/dev/jonas_esp32), BAUD (115200), UROS_WS (~/uros_ws), ROS_DOMAIN_ID (7)
set -e
PORT=${PORT:-/dev/jonas_esp32}
BAUD=${BAUD:-115200}
UROS_WS=${UROS_WS:-$HOME/uros_ws}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}

source /opt/ros/jazzy/setup.bash
source "$UROS_WS/install/local_setup.bash"

[ -e "$PORT" ] || { echo "No existe $PORT: ¿ESP32 conectado? ¿regla udev instalada?" >&2; exit 1; }
echo "micro-ROS Agent en $PORT @ $BAUD (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b "$BAUD" "$@"
