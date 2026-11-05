# Jonas — robot móvil omnidireccional de 3 ruedas (ROS 2 Jazzy + micro-ROS)

**Jonas** es un robot móvil omnidireccional de 3 ruedas omni a 120°.

- **Cerebro:** Raspberry Pi 5 con **Ubuntu 24.04 MATE** y **ROS 2 Jazzy**.
- **Control de bajo nivel:** **ESP32-S3** con **micro-ROS**, que corre el PI de velocidad de cada motor a 100 Hz y publica los ticks de los encoders.
- **Cinemática y odometría:** en la Raspi, en ROS 2 (Python), para cambiar la geometría sin reflashear.
- **Comunicación:** el ESP32 y la Raspi se comunican por **un solo cable USB** (serial), a través del **micro-ROS Agent**.

```
 Laptop (desarrollo)  ── WiFi / SSH ──►  Raspberry Pi 5                ── USB ──►  ESP32-S3
 - código + ESP-IDF                       Ubuntu 24.04 MATE · ROS 2 Jazzy            micro-ROS (Jazzy)
 - compila el firmware                    - micro-ROS Agent                          - PI x3 @ 100 Hz
 - flashea a través de la Raspi           - esptool (flasheo remoto)                 - ticks de encoder @ 50 Hz
                                          - (futuro) cinemática, odometría, Nav2     - encoders (PCNT)
                                                                                         │
                                                                     2x Cytron MDD10A ── 3x motor Pololu 12 V + encoder
```

## Estado

| Parte | Estado |
|---|---|
| Firmware ESP32-S3: micro-ROS Jazzy por serial, reconexión automática, watchdog de comandos | ✅ |
| PI de velocidad en las 3 ruedas (t90 ≈ 0.25 s, sobrepico ≤ 1 %, error ≈ 0, ruedas en el aire) | ✅ |
| Ticks de encoder para odometría (`/jonas/wheel_ticks`, 50 Hz, robustos a pérdida de mensajes) | ✅ |
| Cinemática (`/cmd_vel` → ruedas) y odometría (`/odom`, TF) en la Raspi, en Python | ⏳ (siguiente) |
| micro-ROS Agent en la Raspi 5 | ✅ |
| Flasheo remoto del ESP32 a través de la Raspi (sin cable a la laptop) | ✅ |
| Diagnóstico por ROS (`/jonas/status`) | ✅ |
| Medidas reales (radio de rueda, distancia al centro) y pruebas en el piso | ⏳ |
| URDF, launch, arranque automático (systemd) | ⏳ |

## Contenido del repositorio

```
.
├── firmware/jonas_firmware/        Firmware ESP32-S3 (ESP-IDF 5.5 + micro-ROS Jazzy)
│   ├── main/                       main.c, PI, encoders, motores, transporte UART
│   │   └── jonas_hw.h              ← pines, CPR, inversiones, ganancias PI, nº de motores
│   ├── components/micro_ros_espidf_component/   (submódulo git, rama jazzy)
│   ├── app-colcon.meta             ajustes de micro-ROS (¡importantes para serial!)
│   ├── sdkconfig.defaults          configuración de ESP-IDF
│   ├── idf.sh                      idf.py en entorno limpio (sin variables de ROS)
│   ├── tools/flash_via_pi.sh       compila en la laptop y flashea a través de la Raspi
│   ├── tools/step_test.sh          prueba de escalones del PI + análisis
│   └── README.md                   detalles del firmware
└── raspi/
    ├── 99-jonas.rules              regla udev: /dev/jonas_esp32
    └── run_agent.sh                lanza el micro-ROS Agent
```

---

## Hardware

| Componente | Detalle |
|---|---|
| Computadora principal | Raspberry Pi 5 (4 GB o más) con Ubuntu 24.04 MATE (arm64) |
| Microcontrolador | ESP32-S3-DevKitC-1 (16 MB de flash, 8 MB de PSRAM octal) |
| Drivers de motor | 2x Cytron **MDD10A** (doble canal, PWM + DIR, hasta 20 kHz) |
| Motores | 3x Pololu 12 V con encoder en cuadratura (**6533 cuentas por vuelta de la rueda**) |
| Ruedas | 3x ruedas omni a 120° |
| Alimentación | 12 V para los motores (separada del USB) |

### Pines del ESP32-S3

| Rueda | Posición | Driver | PWM | DIR | Encoder A | Encoder B |
|---|---|---|---|---|---|---|
| 1 | 0° (frente, define +x) | MDD10A #1, canal 1 | GPIO 4 | GPIO 5 | GPIO 2 | GPIO 42 |
| 2 | 120° (atrás izquierda) | MDD10A #1, canal 2 | GPIO 6 | GPIO 7 | GPIO 41 | GPIO 40 |
| 3 | −120° (atrás derecha) | MDD10A #2, canal 1 | GPIO 15 | GPIO 16 | GPIO 39 | GPIO 38 |

Otros: I2C para una IMU futura (SDA 8, SCL 9), voltaje de batería con divisor (GPIO 1, ADC1).
Pines libres: 10–14, 17, 18, 21 y 47.

> ⚠️ **Antes de cablear:**
> - **Los GPIO del ESP32 aceptan como máximo 3.3 V.** Alimenta los encoders con 3.3 V; si los alimentas con 5 V, sus salidas también son de 5 V y necesitas un conversor de nivel.
> - **Tierra común:** GND del ESP32, de los dos MDD10A y de la fuente de los motores.
> - **Nunca alimentes los motores desde el USB.**
> - Haz las primeras pruebas **con las ruedas en el aire**.

**Conexión ESP32 ↔ Raspi:** un solo cable USB al puerto **"UART"** de la DevKitC (chip CH343, `1a86:55d3`). Ese puerto sirve para micro-ROS, para flashear (el CH343 reinicia el ESP32 automáticamente con DTR/RTS) y para recuperarlo si un firmware falla. Usa un USB 2.0 de la Raspi y deja los USB 3.0 para la cámara y el lidar.

---

## Instalación

Hay dos equipos:

| Equipo | Para qué | Sistema probado |
|---|---|---|
| **PC de desarrollo** (laptop) | editar y compilar el firmware (ESP-IDF) | Ubuntu 22.04 (cualquier Linux con ESP-IDF 5.5 sirve) |
| **Raspberry Pi 5** | ejecutar ROS 2 y el micro-ROS Agent; flashear el ESP32 | Ubuntu 24.04 MATE + ROS 2 Jazzy |

> No hace falta ROS 2 en la laptop para compilar el firmware. ESP-IDF no se instala en la Raspi, porque ocupa varios GB.

### 1. PC de desarrollo (laptop)

#### 1.1 ESP-IDF v5.5 (~4 GB)

```bash
sudo apt install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache \
                 libffi-dev libssl-dev dfu-util libusb-1.0-0
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5 --recursive --depth 1 --shallow-submodules https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
```

Dependencias de Python que micro-ROS necesita **dentro del entorno de ESP-IDF**:

```bash
. ~/esp/esp-idf/export.sh
pip install catkin_pkg colcon-common-extensions lark empy==3.3.4
```

Permiso para usar el puerto serial (una vez; luego cierra sesión y vuelve a entrar):

```bash
sudo usermod -aG dialout $USER
```

#### 1.2 Clonar el repositorio (con el submódulo de micro-ROS)

```bash
git clone --recursive https://github.com/Migueltoh2309/Jonas_Omni_Robot.git ~/jonas
# si ya lo clonaste sin --recursive:
cd ~/jonas && git submodule update --init
```

#### 1.3 Compilar

```bash
cd ~/jonas/firmware/jonas_firmware
./idf.sh build        # la 1.ª vez compila la librería micro-ROS (~5–10 min)
```

`idf.sh` busca ESP-IDF en `$IDF_DIR`, `$IDF_PATH` o `~/esp/esp-idf`, y lo ejecuta en un entorno limpio, sin variables de ROS 2.

**Para flashear con el ESP32 conectado a la laptop** (la primera vez, o en el banco de pruebas):

```bash
./idf.sh -p /dev/ttyACM0 flash
```

**Para flashear con el ESP32 conectado a la Raspi**, ver [Flasheo remoto](#4-reprogramar-el-esp32-dentro-del-robot-flasheo-remoto).

### 2. Raspberry Pi 5 (Ubuntu 24.04 MATE)

#### 2.1 ROS 2 Jazzy (ros-base)

Resumen de la [guía oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Si algo cambió, sigue la guía oficial.

```bash
sudo apt install software-properties-common curl
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade
sudo apt install ros-jazzy-ros-base ros-dev-tools
sudo rosdep init && rosdep update
```

En `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7      # dominio propio del robot (ver "Problemas conocidos")
```

#### 2.2 micro-ROS Agent (~100 MB, ~7 min en la Raspi 5)

Se clona el Agent directamente; no hace falta `micro_ros_setup` ni paquetes extra:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent
git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git src/micro_ros_msgs
rosdep check --from-paths src --ignore-src     # debe decir: All system dependencies have been satisfied
MAKEFLAGS="-j3" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### 2.3 Permisos del puerto serial y nombre fijo `/dev/jonas_esp32`

```bash
git clone --recursive https://github.com/Migueltoh2309/Jonas_Omni_Robot.git ~/jonas   # o copia solo raspi/
sudo usermod -aG dialout $USER                       # luego cierra sesión y vuelve a entrar
sudo cp ~/jonas/raspi/99-jonas.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
ls -l /dev/jonas_esp32                               # -> ttyACM0
```

#### 2.4 esptool, para flashear el ESP32 desde la Raspi (11 MB, sin sudo)

```bash
python3 -m venv --without-pip --system-site-packages ~/esptool-venv
~/esptool-venv/bin/python -m pip install "esptool>=4.8,<5"
~/esptool-venv/bin/esptool.py version
```

#### 2.5 Acceso SSH por clave desde la laptop

En la **laptop**:

```bash
ssh-copy-id jonas@jonas.local        # usuario@hostname de la Raspi (o usuario@IP)
ssh jonas@jonas.local                # debe entrar sin pedir contraseña
```

---

## 3. Uso

### Lanzar el micro-ROS Agent (en la Raspi)

```bash
~/jonas/raspi/run_agent.sh
```

Salida esperada: `running...` → `session established` → `participant created` → topics/publishers/subscribers creados.
Después se queda quieto; eso es normal.

### Interfaz ROS 2

| Tópico | Tipo | Dir. | Descripción |
|---|---|---|---|
| `/jonas/wheel_vel_cmd` | `std_msgs/Float32MultiArray` | → ESP32 | velocidad de cada rueda [rad/s], con PI (**entrada principal**) |
| `/jonas/wheel_pwm` | `std_msgs/Float32MultiArray` | → ESP32 | duty por rueda [−1..1], **lazo abierto**, para pruebas (máxima prioridad) |
| `/jonas/pid_gains` | `std_msgs/Float32MultiArray` | → ESP32 | `[kp, ki, kd]` o `[kp, ki, kd, kff]` en vivo (se pierden al reiniciar) |
| `/jonas/wheel_state_rate` | `std_msgs/Int32` | → ESP32 | frecuencia de `/jonas/wheel_state` en Hz (50, 25, 10, 5, 2, 1; 0 = apagado) |
| `/jonas/wheel_ticks` | `std_msgs/Int32MultiArray` | ESP32 → | 50 Hz: `[t_ms, ticks1, ticks2, ticks3]`, cuentas acumuladas, para la odometría |
| `/jonas/wheel_state` | `std_msgs/Float32MultiArray` | ESP32 → | 25 Hz por defecto: `[objetivo x3, medida x3, duty x3]`, para depurar |
| `/jonas/status` | `std_msgs/String` | ESP32 → | al conectar (motivo del último arranque) y cada 2 s (uptime, modo, conexiones) |

**Seguridad:** cada comando vale mientras llegue al menos cada **500 ms**. Si deja de llegar, o si se pierde el Agent, los motores se detienen. El ESP32 se reconecta solo.

**Convenciones:** REP-103 (+x adelante hacia la rueda 1, +y izquierda, `wz > 0` = giro antihorario) y unidades en rad/s (1 rad/s = 9.549 RPM). Con velocidad positiva, cada rueda gira en sentido **horario vista desde afuera del robot**.

### Pruebas rápidas (en la Raspi, con las ruedas en el aire)

```bash
ros2 topic echo /jonas/status
ros2 topic echo /jonas/wheel_state --qos-reliability best_effort
ros2 topic pub -r 20 /jonas/wheel_vel_cmd std_msgs/msg/Float32MultiArray "{data: [2.0, 2.0, 2.0]}"   # 2 rad/s cada rueda
ros2 topic echo /jonas/wheel_ticks --qos-reliability best_effort                                   # ticks acumulados
~/jonas/firmware/jonas_firmware/tools/step_test.sh          # escalones ±20/±50 RPM + tabla por rueda
```

## 4. Reprogramar el ESP32 dentro del robot (flasheo remoto)

El ESP32 **no necesita conectarse a la laptop**. Desde la laptop:

```bash
cd ~/jonas/firmware/jonas_firmware
tools/flash_via_pi.sh                         # compila, copia por SSH y flashea a través de la Raspi (~13 s)
PI=jonas@192.168.1.50 tools/flash_via_pi.sh   # si jonas.local no resuelve, usa la IP
```

El script detiene el Agent de la Raspi (el puerto no puede estar ocupado), flashea con `esptool` y, si el Agent estaba corriendo, lo relanza en segundo plano (log en `~/jonas_fw/agent.log` de la Raspi). La ROM del ESP32 siempre acepta firmware nuevo, así que **un firmware con errores no lo inutiliza**.

## 5. Configuración

| Qué | Dónde |
|---|---|
| Pines, CPR, inversión de motor o encoder, nº de motores, ganancias PI, filtro | `firmware/jonas_firmware/main/jonas_hw.h` |
| Baudrate, `ROS_DOMAIN_ID`, frecuencia de control, timeout de comandos | `./idf.sh menuconfig` → menú **Jonas** |
| Geometría (radio de rueda, distancia al centro, ángulos 0°/120°/240°) | parámetros del nodo de cinemática y odometría en la Raspi (próximamente) |
| Límites de micro-ROS (entidades, heartbeat, buffers) | `firmware/jonas_firmware/app-colcon.meta` (luego `./idf.sh clean-microros`) |

Más detalles del firmware (PI, modelo del motor, sintonía): [`firmware/jonas_firmware/README.md`](firmware/jonas_firmware/README.md).

## 6. Problemas conocidos y soluciones

| Síntoma | Causa | Solución |
|---|---|---|
| El micro-ROS Agent **Humble** aborta con `fastcdr::exception::NotEnoughMemoryException` | otra máquina con ROS 2 **Jazzy** en el mismo `ROS_DOMAIN_ID` y la misma red (Humble y Jazzy no son compatibles en DDS) | usar dominios distintos: robot en **7**; firmware con `JONAS_ROS_DOMAIN_ID` igual al del Agent |
| No aparece ningún tópico del ESP32 | el dominio del firmware es distinto al de tu terminal | el dominio del ESP32 lo fija el **firmware**, no la variable de entorno del Agent |
| El Agent solo muestra `running...` | el ESP32 no responde | botón RST del ESP32, o reflashear |
| `Permission denied` en `/dev/ttyACM0` | el usuario no está en `dialout` | `sudo usermod -aG dialout $USER` y volver a iniciar sesión |
| Falla la creación de entidades / desconexiones al agregar tópicos | heartbeats de XRCE cada 1 ms saturan el serial a 115200 | ya resuelto en `app-colcon.meta` (heartbeat de 50 ms, historial de stream de 8); no revertir |
| `/jonas_base` aparece duplicado | caché del daemon de ROS 2 | `ros2 daemon stop` |

## 7. Próximos pasos

- [ ] Nodo de cinemática y odometría en Python (`ros2_ws/src/jonas_base`): `/cmd_vel` → `/jonas/wheel_vel_cmd` y `/jonas/wheel_ticks` → `/odom` + TF
- [ ] Medidas reales (radio de rueda, distancia al centro) y prueba de `/cmd_vel` en el piso
- [ ] Re-sintonizar el PI con carga (robot en el piso)
- [ ] URDF y launch file
- [ ] Arranque automático con systemd
- [ ] IMU, cámara, lidar
- [ ] Navigation 2

---

## Licencia

[MIT](LICENSE) © 2026 MiTo Olortegui Huaman — UTEC.

El componente `micro_ros_espidf_component` (submódulo) tiene su propia licencia (Apache-2.0).
