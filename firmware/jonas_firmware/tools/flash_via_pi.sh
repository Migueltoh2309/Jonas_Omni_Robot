#!/usr/bin/env bash
# Compila el firmware en la laptop y flashea el ESP32 conectado a la Raspberry Pi (por SSH).
# No hace falta conectar el ESP32 a la laptop: usa el puerto UART (CH343) del ESP32 en la Raspi.
#
# Uso:   tools/flash_via_pi.sh            # compila + flashea
#        tools/flash_via_pi.sh --no-build # flashea el último build
# Variables (opcionales):
#   PI=jonas@jonas.local      destino SSH (usuario@hostname o usuario@IP de la Raspi)
#   PORT=/dev/jonas_esp32     puerto del ESP32 en la Raspi (regla udev raspi/99-jonas.rules)
#   BAUD=460800               velocidad de flasheo
# Requisitos: acceso SSH por clave a la Raspi (ssh-copy-id) y esptool en ~/esptool-venv de la Raspi.
set -euo pipefail

PI=${PI:-jonas@jonas.local}
PORT=${PORT:-/dev/jonas_esp32}
BAUD=${BAUD:-460800}
REMOTE_DIR='~/jonas_fw'
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new"

cd "$(dirname "$0")/.."

if [ "${1:-}" != "--no-build" ]; then
    echo "==> Compilando en la laptop"
    ./idf.sh build | tail -3
fi

B=build
for f in flash_args bootloader/bootloader.bin partition_table/partition-table.bin jonas_firmware.bin; do
    [ -f "$B/$f" ] || { echo "Falta $B/$f (¿compilaste?)"; exit 1; }
done

echo "==> Copiando binarios a $PI:$REMOTE_DIR"
tar -C "$B" -cf - flash_args bootloader/bootloader.bin partition_table/partition-table.bin jonas_firmware.bin \
    | $SSH "$PI" "rm -rf $REMOTE_DIR && mkdir -p $REMOTE_DIR && tar -C $REMOTE_DIR -xf -"

echo "==> Flasheando $PORT en la Raspi"
# El Agent tiene el puerto abierto: se detiene, se flashea y se vuelve a levantar si estaba corriendo.
# ([m]icro... evita que pgrep/pkill se encuentren a sí mismos en la línea de comandos.)
$SSH "$PI" bash -s -- "$PORT" "$BAUD" <<'REMOTE' 2>&1 | grep -v setlocale
set -e
PORT=$1; BAUD=$2
[ -e "$PORT" ] || { echo "ERROR: no existe $PORT (¿ESP32 conectado a la Raspi?)"; exit 1; }
AGENT_CMD=$(pgrep -af '[m]icro_ros_agent serial' | head -1 | cut -d' ' -f2- || true)
if [ -n "$AGENT_CMD" ]; then
    echo "    deteniendo micro_ros_agent"
    pkill -f '[m]icro_ros_agent serial' || true
    sleep 1
fi
cd ~/jonas_fw
RC=0
~/esptool-venv/bin/esptool.py --chip esp32s3 -p "$PORT" -b "$BAUD" \
    --before default_reset --after hard_reset write_flash @flash_args > esptool.log 2>&1 || RC=$?
grep -E "Wrote|Hash of data|Hard resetting|rror" esptool.log | sed 's/^/    /' || true
if [ "$RC" -ne 0 ]; then echo "ERROR: esptool falló (código $RC). Detalle: ~/jonas_fw/esptool.log"; fi
if [ -n "$AGENT_CMD" ]; then
    echo "    relanzando micro_ros_agent en segundo plano (log: ~/jonas_fw/agent.log)"
    source /opt/ros/jazzy/setup.bash
    source ~/uros_ws/install/local_setup.bash
    nohup ros2 run micro_ros_agent micro_ros_agent serial --dev "$PORT" -b 115200 > ~/jonas_fw/agent.log 2>&1 &
fi
exit "$RC"
REMOTE
echo "==> Listo"
