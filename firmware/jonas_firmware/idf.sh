#!/usr/bin/env bash
# Ejecuta idf.py en un entorno limpio, sin variables de ROS 2 (si ~/.bashrc carga otra distro de ROS,
# p. ej. Humble, sus paquetes podrían mezclarse al compilar micro-ROS Jazzy).
# Uso: ./idf.sh build | ./idf.sh -p /dev/ttyACM0 flash | ./idf.sh menuconfig
#
# Ubicación de ESP-IDF v5.5 (en este orden): $IDF_DIR, $IDF_PATH, ~/esp/esp-idf
set -e
IDF_DIR="${IDF_DIR:-${IDF_PATH:-$HOME/esp/esp-idf}}"
if [ ! -f "$IDF_DIR/export.sh" ]; then
    echo "No encuentro ESP-IDF en '$IDF_DIR'. Instálalo en ~/esp/esp-idf o exporta IDF_DIR=/ruta/a/esp-idf" >&2
    exit 1
fi
cd "$(dirname "$0")"
exec env -i HOME="$HOME" USER="$USER" TERM="${TERM:-xterm}" LANG="${LANG:-C.UTF-8}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    bash --noprofile --norc -c 'source "$0/export.sh" >/dev/null && idf.py "$@"' "$IDF_DIR" "$@"
