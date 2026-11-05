#!/usr/bin/env bash
# Prueba de escalones de velocidad (misma referencia en las 3 ruedas) y análisis por rueda.
# Requiere el micro_ros_agent corriendo. RUEDA EN EL AIRE.
# Uso: ./step_test.sh [kp ki]   (si se dan, se envían en vivo por /jonas/pid_gains)
#      Escalones en RPM: variable RPMS (por defecto "20 -20 50 -50").
set -e
cd "$(dirname "$0")"
RPMS=${RPMS:-"20 -20 50 -50"}
OUT=${OUT:-step_$(date +%H%M%S).csv}
if [ -n "$2" ]; then
  ros2 topic pub --once /jonas/pid_gains std_msgs/msg/Float32MultiArray "{data: [$1, $2, 0.0]}" >/dev/null
  sleep 1
fi
N=$(echo $RPMS | wc -w)
# wheel_state a 50 Hz durante la prueba (el análisis asume 50 Hz); al final vuelve a 25 Hz.
ros2 topic pub --once /jonas/wheel_state_rate std_msgs/msg/Int32 "{data: 50}" >/dev/null
trap 'ros2 topic pub --once /jonas/wheel_state_rate std_msgs/msg/Int32 "{data: 25}" >/dev/null 2>&1' EXIT
timeout $((N * 4 + 3)) ros2 topic echo --csv --qos-reliability best_effort /jonas/wheel_state \
  std_msgs/msg/Float32MultiArray > "$OUT" 2>/dev/null & EP=$!
sleep 1
for rpm in $RPMS; do
  w=$(python3 -c "print(round($rpm * 0.104720, 4))")
  timeout 3.5 ros2 topic pub -r 20 /jonas/wheel_vel_cmd std_msgs/msg/Float32MultiArray "{data: [$w, $w, $w]}" >/dev/null 2>&1 || true
done
wait $EP 2>/dev/null || true
echo "Datos: $OUT  (columnas: offset, objetivo x3, medida x3, duty x3)"
python3 step_analysis.py "$OUT"
