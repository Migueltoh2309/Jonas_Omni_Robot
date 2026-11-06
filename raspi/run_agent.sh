#!/usr/bin/env bash
# Lanza el micro-ROS Agent (ROS 2 Jazzy) para el ESP32 de Jonas, en la Raspberry Pi.
# Uso: raspi/run_agent.sh            (Ctrl+C para detener)
# Variables opcionales: PORT (/dev/jonas_esp32), BAUD (115200), UROS_WS (~/uros_ws), ROS_DOMAIN_ID (7),
#                       RESET_ESP32 (1: reinicia el ESP32 antes de lanzar el Agent; 0: no)
set -e
PORT=${PORT:-/dev/jonas_esp32}
BAUD=${BAUD:-115200}
UROS_WS=${UROS_WS:-$HOME/uros_ws}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7}

source /opt/ros/jazzy/setup.bash
source "$UROS_WS/install/local_setup.bash"

[ -e "$PORT" ] || { echo "No existe $PORT: ¿ESP32 conectado? ¿regla udev instalada?" >&2; exit 1; }
# Reiniciar el ESP32 (pin EN vía RTS del CH343, con IO0 en alto = arranque normal) para que se conecte
# enseguida a este Agent, aunque haya arrancado antes que él.
if [ "${RESET_ESP32:-1}" = "1" ]; then
    python3 - "$PORT" <<'PY' && echo "ESP32 reiniciado" || echo "Aviso: no se pudo reiniciar el ESP32" >&2
import sys, time, serial
s = serial.Serial(sys.argv[1])
s.dtr = False      # IO0 en alto (no entrar al bootloader)
s.rts = True       # EN en bajo (reset)
time.sleep(0.1)
s.rts = False      # suelta EN: arranca el firmware
s.close()
PY
    sleep 0.5
fi
echo "micro-ROS Agent en $PORT @ $BAUD (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b "$BAUD" "$@"
