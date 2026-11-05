# jonas_firmware — ESP32-S3 + micro-ROS (Jazzy) por serial

Firmware de bajo nivel del robot **Jonas**. Hace el control rápido de cada rueda: lee los encoders,
corre un PI de velocidad por rueda a 100 Hz y maneja los motores. Se comunica con ROS 2 Jazzy (Raspberry Pi 5)
a través del `micro_ros_agent` por USB serial.

**La cinemática (`/cmd_vel` → velocidades de rueda) y la odometría (ticks → `/odom`) se hacen en la Raspi**,
en ROS 2. Así, la geometría del robot se cambia sin reflashear.

| Componente | Versión |
|---|---|
| ESP-IDF | v5.5 (en `~/esp/esp-idf`; `idf.sh` también acepta `$IDF_DIR` / `$IDF_PATH`) |
| micro-ROS | `micro_ros_espidf_component`, rama `jazzy` (submódulo en `components/`) |
| Transporte | XRCE-DDS sobre UART0 (puerto "UART"/CH343 de la DevKitC), 115200 baud |
| Dominio ROS | `CONFIG_JONAS_ROS_DOMAIN_ID=7` (en `sdkconfig.defaults`) |

## Interfaz ROS 2

| Tópico | Tipo | Dir. | Descripción |
|---|---|---|---|
| `/jonas/wheel_vel_cmd` | `std_msgs/Float32MultiArray` | sub | velocidad por rueda `[w1, w2, w3]` en rad/s, con PI. **Entrada principal** |
| `/jonas/wheel_pwm` | `std_msgs/Float32MultiArray` | sub | duty por rueda [−1..1], **lazo abierto**, para pruebas (prioridad sobre `wheel_vel_cmd`) |
| `/jonas/pid_gains` | `std_msgs/Float32MultiArray` | sub | `[kp, ki, kd]` o `[kp, ki, kd, kff]` en vivo (se pierde al reiniciar) |
| `/jonas/wheel_state_rate` | `std_msgs/Int32` | sub | frecuencia de `/jonas/wheel_state` en Hz (divisor de 50: 50, 25, 10, 5, 2, 1; 0 = apagado) |
| `/jonas/wheel_ticks` | `std_msgs/Int32MultiArray` | pub | **50 Hz**, best effort: `[t_ms, ticks1, ticks2, ticks3]`, para la odometría |
| `/jonas/wheel_state` | `std_msgs/Float32MultiArray` | pub | **25 Hz** por defecto, best effort: `[objetivo x3, medida x3, duty x3]`, para depurar |
| `/jonas/status` | `std_msgs/String` | pub | al conectar (motivo del último arranque) y cada 2 s (uptime, modo, conexiones, fallos, frecuencia) |

### `/jonas/wheel_ticks` (odometría)

- `t_ms`: reloj del ESP32 en milisegundos, tomado **en el mismo instante** en que se leyeron los encoders.
- `ticksN`: cuenta **acumulada** del encoder de la rueda N desde el arranque del ESP32 (entero exacto, 6533 por vuelta de rueda, con el signo ya corregido).
- **Aunque se pierdan mensajes**, la diferencia de ticks entre dos mensajes da la distancia exacta recorrida. Para la velocidad, usa `Δticks / Δt_ms`.
- Rad = ticks × 2π / 6533. Los ticks vuelven a 0 si el ESP32 se reinicia (detéctalo con `/jonas/status` o con un salto de `t_ms` hacia atrás).

### Seguridad

- Cada comando de rueda vale **500 ms** (`CONFIG_JONAS_CMD_TIMEOUT_MS`). Si deja de llegar, o si se pierde el Agent, los motores se detienen.
- Las velocidades se limitan a `JONAS_WHEEL_MAX_RADPS` (10 rad/s). Si una rueda lo supera, las tres se escalan por igual para conservar la dirección del movimiento.
- Al arrancar, los pines de los motores se ponen en bajo antes que cualquier otra cosa.
- Reconexión automática al Agent.

## Hardware y configuración (`main/jonas_hw.h`)

Cytron MDD10A (PWM 20 kHz + DIR) y motores Pololu 12 V con encoder de 6533 cuentas por vuelta de rueda.

| Rueda | Posición | PWM | DIR | Enc A | Enc B | invert_motor | invert_encoder |
|---|---|---|---|---|---|---|---|
| 1 | 0° (frente) | 4 | 5 | 2 | 42 | false | false |
| 2 | 120° | 6 | 7 | 41 | 40 | true | false |
| 3 | −120° | 15 | 16 | 39 | 38 | true | true |

Las inversiones están verificadas: con velocidad positiva, las 3 ruedas giran en sentido **horario vistas desde
afuera del robot** y cada encoder cuenta en el mismo sentido que su motor. `invert_motor` e `invert_encoder`
son **independientes**: si inviertes un motor, invierte también su encoder, o el PI queda con
realimentación positiva y se va al máximo.

## Control de velocidad (PI)

PI por rueda a 100 Hz, con media móvil de 7 muestras y anti-windup condicional (la misma estructura que
`Extra/03_control_PI_velocidad.ino` del repo [SwarmBot_microROS](https://github.com/Migueltoh2309/SwarmBot_microROS)).

Modelo identificado (rueda en el aire, 12 V): `G(s) = K/(τs+1)` con K ≈ 11.8 rad/s por unidad de duty,
τ ≈ 0.08 s y zona muerta ≈ 4 %. Sintonía por cancelación de polo (Ti = τ, Tc = 0.1 s):
**Kp = 0.068, Ki = 0.85**. Resultado: t90 ≈ 0.25 s, sobrepico ≤ 1 %, error en régimen ≈ 0 en las 3 ruedas.
Hay que re-verificarlo con el robot en el piso.

```bash
# en la Raspi, con el Agent corriendo y LAS RUEDAS EN EL AIRE
ros2 topic pub -r 20 /jonas/wheel_pwm std_msgs/msg/Float32MultiArray "{data: [0.15, 0.15, 0.15]}"    # lazo abierto
ros2 topic pub -r 20 /jonas/wheel_vel_cmd std_msgs/msg/Float32MultiArray "{data: [2.0, 2.0, 2.0]}"   # 2 rad/s con PI
ros2 topic pub --once /jonas/pid_gains std_msgs/msg/Float32MultiArray "{data: [0.068, 0.85, 0.0]}"   # ganancias en vivo
ros2 topic pub --once /jonas/wheel_state_rate std_msgs/msg/Int32 "{data: 50}"                         # wheel_state a 50 Hz
tools/step_test.sh               # escalones ±20/±50 RPM + tabla por rueda (pone wheel_state a 50 Hz y lo devuelve a 25)
tools/step_test.sh 0.08 1.0      # misma prueba con otras ganancias (en vivo, no se graban)
```

Unidades: rad/s (estándar ROS). 1 rad/s = 9.549 RPM.

## Compilar y flashear

```bash
cd ~/jonas/firmware/jonas_firmware
./idf.sh build
tools/flash_via_pi.sh             # flashea a través de la Raspi (ESP32 conectado a la Raspi), ~13 s
tools/flash_via_pi.sh --no-build  # reflashea el último build
./idf.sh -p /dev/ttyACM0 flash    # si el ESP32 está conectado directamente a la laptop
./idf.sh menuconfig               # menú "Jonas": baudrate, ROS_DOMAIN_ID, frecuencia de control, timeout
```

`idf.sh` ejecuta `idf.py` en un entorno limpio, sin variables de ROS 2 que puedan mezclarse al compilar micro-ROS.

**Un solo cable basta.** El puerto "UART" (CH343) sirve para micro-ROS, para flashear (el CH343 reinicia el
ESP32 en modo programación con DTR/RTS) y para recuperarlo: la ROM siempre acepta firmware nuevo.
Los logs (`ESP_LOG`) salen por el puerto USB nativo, que es opcional; para diagnóstico con un solo cable,
usa `/jonas/status`.

## Ajustes de micro-ROS (`app-colcon.meta`) — no revertir

Con serial a 115200 y 7 entidades, los valores por defecto del componente fallan. Estos ajustes son necesarios:

| Ajuste | Valor | Por qué |
|---|---|---|
| `UCLIENT_MIN_HEARTBEAT_TIME_INTERVAL` | 50 ms (el componente trae 1 ms) | con 1 ms los heartbeats saturan el serial y la creación de entidades falla por timeout |
| `RMW_UXRCE_STREAM_HISTORY_OUTPUT` | 16 | con un historial menor que el nº de entidades, `destroy_entities()` se cuelga cuando el Agent se cae |
| `RMW_UXRCE_STREAM_HISTORY_INPUT` | 8 | margen para varias suscripciones |
| `RMW_UXRCE_MAX_SUBSCRIPTIONS` / `PUBLISHERS` | 4 / 6 | entidades del firmware |

Además, en el código: el `rcl_allocator_t` que se pasa a rclc **debe ser estático**, porque rclc guarda el
puntero. Si es una variable local, el ESP32 hace panic (`InstrFetchProhibited`) en el primer `spin_some`.

Tras cambiar `app-colcon.meta`: `./idf.sh clean-microros` y recompilar.
