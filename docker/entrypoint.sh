#!/bin/bash
# Carga ROS, compila el workspace de Jonas (montado en ~/ws/src) si cambió algo, y ejecuta el comando.
set -e
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ws
if [ -d src ] && [ "${JONAS_BUILD:-1}" = "1" ]; then
    colcon build --symlink-install --packages-ignore jonas_vision \
        --event-handlers console_cohesion- status- summary+ >/tmp/colcon.log 2>&1 \
        || { cat /tmp/colcon.log; echo "Falló colcon build" >&2; exit 1; }
    grep -E "Summary|Failed" /tmp/colcon.log || true
fi
[ -f install/setup.bash ] && source install/setup.bash
exec "$@"
