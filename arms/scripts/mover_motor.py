#!/usr/bin/env python3
"""Código base: mover UN Dynamixel (MX-28 / serie X) con el Dynamixel SDK.

Mueve el motor un poco hacia un lado, luego hacia el otro, y vuelve a la posición inicial,
a velocidad baja. Al final apaga el torque (queda igual que al empezar).

Uso (en la Raspi, con la fuente de los servos encendida):
    python3 mover_motor.py --id 3 --proto 1          # MX-28 con Protocolo 1.0
    python3 mover_motor.py --id 3 --proto 2          # Protocolo 2.0 (MX-28 2.0, XC330, ...)
    python3 mover_motor.py --id 3 --delta 300        # recorrido más grande (pasos; 4096 = 1 vuelta)

⚠️ Verifica que el brazo tenga espacio libre para moverse ±(delta * 360/4096) grados.
"""
import argparse
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# Direcciones de la tabla de control (cambian según el protocolo)
TABLE = {
    1: dict(torque=24, goal=30, goal_size=2, speed=32, speed_size=2, present=36, present_size=2, moving=46),
    2: dict(torque=64, goal=116, goal_size=4, speed=112, speed_size=4, present=132, present_size=4, moving=122),
}
# Unidad de velocidad: P1.0 (Moving Speed) = 0.114 rpm/unidad; P2.0 (Profile Velocity) = 0.229 rpm/unidad
RPM_PER_UNIT = {1: 0.114, 2: 0.229}


def main():
    ap = argparse.ArgumentParser(description="Mover un Dynamixel de forma segura")
    ap.add_argument("--port", default="/dev/jonas_u2d2")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--proto", type=int, choices=(1, 2), default=2)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--delta", type=int, default=150, help="recorrido en pasos a cada lado (150 ≈ 13°)")
    ap.add_argument("--rpm", type=float, default=7.0, help="velocidad máxima del movimiento")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="si > 0, va y vuelve entre los extremos durante ese tiempo (s)")
    args = ap.parse_args()

    t = TABLE[args.proto]
    port = PortHandler(args.port)
    ph = PacketHandler(1.0 if args.proto == 1 else 2.0)
    if not (port.openPort() and port.setBaudRate(args.baud)):
        raise SystemExit(f"No se pudo abrir {args.port} a {args.baud} bps")

    def check(result, error, what):
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{what}: {ph.getTxRxResult(result)}")
        if error:
            raise RuntimeError(f"{what}: {ph.getRxPacketError(error)}")

    def write(addr, size, value, what):
        fn = {1: ph.write1ByteTxRx, 2: ph.write2ByteTxRx, 4: ph.write4ByteTxRx}[size]
        check(*fn(port, args.id, addr, value), what)

    def read(addr, size, what):
        fn = {1: ph.read1ByteTxRx, 2: ph.read2ByteTxRx, 4: ph.read4ByteTxRx}[size]
        value, result, error = fn(port, args.id, addr)
        check(result, error, what)
        return value

    def go_to(goal):
        write(t["goal"], t["goal_size"], goal, "escribir posición objetivo")
        t0 = time.time()
        while time.time() - t0 < 10:
            time.sleep(0.05)
            pos = read(t["present"], t["present_size"], "leer posición")
            if abs(pos - goal) <= 5 and not read(t["moving"], 1, "leer moving"):
                break
        print(f"  objetivo {goal:5d}  ->  llegó a {pos:5d} ({pos * 360 / 4096:6.1f}°)")

    try:
        model, result, error = ph.ping(port, args.id)
        check(result, error, f"ping ID {args.id}")
        start = read(t["present"], t["present_size"], "leer posición")
        print(f"ID {args.id} (modelo {model}) en {start} ({start * 360 / 4096:.1f}°)")

        speed = max(1, round(args.rpm / RPM_PER_UNIT[args.proto]))   # nunca 0: en P1.0, 0 = velocidad máxima
        if args.proto == 2:
            write(t["torque"], 1, 0, "apagar torque")                # Profile Velocity solo se escribe sin torque
        write(t["speed"], t["speed_size"], speed, "limitar velocidad")
        write(t["torque"], 1, 1, "encender torque")

        lo, hi = max(0, start - args.delta), min(4095, start + args.delta)
        if args.seconds > 0:
            t_end = time.time() + args.seconds
            while time.time() < t_end:
                go_to(hi)
                if time.time() >= t_end:
                    break
                go_to(lo)
            go_to(start)
        else:
            for goal in (hi, lo, start):
                go_to(goal)
    finally:
        try:
            write(t["torque"], 1, 0, "apagar torque")
        except Exception as e:  # noqa: BLE001
            print("Aviso: no se pudo apagar el torque:", e)
        port.closePort()
        print("Torque apagado. Listo.")


if __name__ == "__main__":
    main()
