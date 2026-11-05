# jonas_firmware — ESP32-S3 + micro-ROS (Jazzy) por serial

Firmware base del robot omnidireccional de 3 ruedas **Jonas**. El ESP32-S3 corre el
control de bajo nivel y habla con ROS 2 Jazzy (Raspberry Pi 5) a través del
`micro_ros_agent` por USB serial.

| Componente | Versión |
|---|---|
| ESP-IDF | v5.5 (en `~/esp/esp-idf`; `idf.sh` también acepta `$IDF_DIR` / `$IDF_PATH`) |
| micro-ROS | `micro_ros_espidf_component`, rama `jazzy` (en `components/`) |
| Transporte | XRCE-DDS sobre UART0, 115200 baud |

## Puertos de la DevKitC-1

- **Puerto "UART" (CH343, `1a86:55d3`)** → micro-ROS. Va a la Raspi. También se usa para flashear.
- **Puerto "USB" (USB nativo, `303a:1001`)** → logs (`ESP_LOG`). Opcional, solo para depurar.

Los logs NO salen por el puerto UART para no corromper el stream de micro-ROS.
Solo los mensajes de la ROM al arrancar salen por UART0, y el Agent los ignora.

## Interfaz ROS

| Tópico | Tipo | Dir | Nota |
|---|---|---|---|
| `/jonas/wheel_pwm` | `std_msgs/Float32MultiArray` | sub | duty por rueda [-1..1], **lazo abierto** (pruebas). Prioridad 1 |
| `/jonas/wheel_vel_cmd` | `std_msgs/Float32MultiArray` | sub | velocidad por rueda [rad/s], con PID. Prioridad 2 |
| `/cmd_vel` | `geometry_msgs/Twist` | sub | `linear.x`, `linear.y`, `angular.z` → cinemática → PID. Prioridad 3 |
| `/jonas/pid_gains` | `std_msgs/Float32MultiArray` | sub | `[kp, ki, kd]` o `[kp, ki, kd, kff]`, en vivo (se pierde al reiniciar) |
| `/jonas/wheel_state` | `std_msgs/Float32MultiArray` | pub | 50 Hz, best effort: `[objetivo x3, medida x3, duty x3]` (rad/s, rad/s, -1..1) |
| `/jonas/status` | `std_msgs/String` | pub | al conectar (motivo de arranque) y cada 2 s (uptime, modo, conexiones, fallos) |

Seguridad: cada comando vale mientras llegue al menos cada 500 ms; si no, o sin Agent → motores a 0.
Se reconecta solo. Hardware (pines, CPR, inversiones, PID por defecto, nº de motores) en `main/jonas_hw.h`.

Hardware actual: Cytron MDD10A (PWM 20 kHz + DIR), motor Pololu 12 V con encoder 6533 CPR (rueda).

| Rueda | PWM | DIR | Enc A | Enc B |
|---|---|---|---|---|
| 1 | 4 | 5 | 2 | 42 |
| 2 | 6 | 7 | 41 | 40 |
| 3 | 15 | 16 | 39 | 38 |

> ⚠️ **Humble y Jazzy en la misma red:** si la Raspi (Jazzy) tiene cualquier cosa de ROS 2 corriendo
> (incluso el `ros2 daemon`) en el mismo `ROS_DOMAIN_ID`, el Agent **Humble** de la laptop se cae con
> `fastcdr::exception::NotEnoughMemoryException`. Por eso:
>
> | Dónde | ROS_DOMAIN_ID |
> |---|---|
> | Laptop (Humble) + ESP32 en el banco de pruebas | 0 (firmware: `JONAS_ROS_DOMAIN_ID=0`) |
> | Raspi (Jazzy), en su `~/.bashrc` | **7** |
>
> **Al pasar el ESP32 a la Raspi:** `./idf.sh menuconfig` → Jonas → `ROS_DOMAIN_ID` = **7**, y reflashear.
> El dominio del ESP32 lo fija el firmware, no la variable de entorno del Agent.

## Compilar y flashear (laptop)

`~/.bashrc` carga ROS 2 Humble, así que se usa `./idf.sh` (corre `idf.py` en un entorno limpio):

```bash
cd ~/jonas/firmware/jonas_firmware
./idf.sh build
./idf.sh -p /dev/ttyACM0 flash
./idf.sh menuconfig        # menú "Jonas": radio de rueda, distancia al centro, baudrate, domain id...
```

Si cambias `app-colcon.meta` o la rama de micro-ROS: `./idf.sh clean-microros` y recompila.

## Reprogramar con el ESP32 dentro del robot (un solo cable, sin conectarlo a la laptop)

```
Laptop ── WiFi/SSH ──► Raspberry Pi 5 ── USB (puerto "UART" del ESP32) ──► ESP32-S3
(código y compilación)  (Agent + esptool)                                   (firmware)
```

Un solo cable USB (puerto **UART**/CH343) basta para todo: micro-ROS, flasheo (reset automático por
DTR/RTS) y recuperación (la ROM del ESP32 siempre acepta firmware nuevo, aunque el programa falle).

```bash
cd ~/jonas/firmware/jonas_firmware
tools/flash_via_pi.sh             # compila en la laptop y flashea a través de la Raspi (~13 s)
tools/flash_via_pi.sh --no-build  # reflashea el último build
PI=jonas@jonas.local tools/flash_via_pi.sh   # si cambia la IP de la Raspi
```

El script detiene el micro_ros_agent de la Raspi (el puerto no puede estar ocupado), flashea y, si estaba
corriendo, lo relanza en segundo plano (log en `~/jonas_fw/agent.log` de la Raspi).
En la Raspi: `esptool` en `~/esptool-venv` (instalado sin sudo).

**Logs sin el segundo cable:** el firmware publica `/jonas/status` (`std_msgs/String`): al conectar
(motivo del último arranque: encendido / PANIC / watchdog / brownout…) y cada 2 s (uptime, modo, conexiones,
fallos). En la Raspi: `ros2 topic echo /jonas/status`.

Herramientas de prueba también en la Raspi: `~/jonas_tools/step_test.sh`.

## Raspberry Pi 5 (Ubuntu 24.04 + Jazzy)

Instalado en `~/uros_ws` (96 MB, ~7 min). Se clona el Agent directamente, sin
`micro_ros_setup`: con `ros-jazzy-ros-base` no hace falta ningún paquete apt extra ni sudo.

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent
git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs
rosdep check --from-paths src --ignore-src      # debe decir "All system dependencies have been satisfied"
MAKEFLAGS="-j3" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Permisos y nombre fijo del puerto (una vez, ya hecho en Jonas):

```bash
sudo usermod -aG dialout $USER   # luego cerrar sesión y volver a entrar
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", SYMLINK+="jonas_esp32", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-jonas.rules
sudo udevadm control --reload && sudo udevadm trigger   # -> /dev/jonas_esp32
```

Correr el Agent:

```bash
source ~/uros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/jonas_esp32 -b 115200
```

Si `ros2 node list` muestra `/jonas_base` duplicado tras reiniciar el Agent, es caché
del daemon de ROS: `ros2 daemon stop` o usar `--no-daemon`.

## Prueba rápida

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
ros2 topic echo /jonas/wheel_cmd --qos-reliability best_effort
# con r=30 mm, L=100 mm -> [-5.77, 0.0, 5.77]; con angular.z=1.0 -> [3.33, 3.33, 3.33]
```

## Control de velocidad (PI) y pruebas

PI por rueda a 100 Hz con media móvil de 7 muestras y anti-windup condicional (estructura de
`swarmbot/Extra/03_control_PI_velocidad.ino`). Ganancias por defecto en `main/jonas_hw.h`
(Kp = 0.068, Ki = 0.85, sintonizadas por cancelación de polo con la rueda en el aire).

Con el Agent corriendo y **la rueda en el aire**:

```bash
ros2 topic pub -r 20 /jonas/wheel_pwm std_msgs/msg/Float32MultiArray "{data: [0.15, 0.0, 0.0]}"   # lazo abierto
ros2 topic pub -r 20 /jonas/wheel_vel_cmd std_msgs/msg/Float32MultiArray "{data: [3.0, 0.0, 0.0]}" # 3 rad/s con PI
ros2 topic pub --once /jonas/pid_gains std_msgs/msg/Float32MultiArray "{data: [0.068, 0.85, 0.0]}" # ganancias en vivo
tools/step_test.sh               # escalones ±20/±50 RPM + tabla (t90, sobrepico, error, ruido)
tools/step_test.sh 0.08 1.0      # misma prueba con otras ganancias (en vivo, no se graban)
```

Unidades: la interfaz usa rad/s (estándar ROS). 1 rad/s = 9.549 RPM.

## Cinemática

Ver `main/omni_kinematics.h`. Ruedas a 60° (del. izq.), 180° (trasera), 300° (del. der.);
+x adelante, velocidad positiva de rueda = giro antihorario visto desde arriba.
Ajusta `OMNI_THETA_DEG` y los parámetros del menú "Jonas" a la geometría real.

## Pendiente

- Drivers de motor + encoders + PID por rueda (`motors_set_targets()` es un stub).
- Publicar velocidad medida y `/odom` (usar `omni_forward()`).
