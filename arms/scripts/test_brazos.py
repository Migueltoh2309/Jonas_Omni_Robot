#!/usr/bin/env python3
"""Prueba de los 6 MX-28 (Protocolo 2.0) de los brazos de Jonas, con Sync Read / Sync Write.

Modos:
    python3 test_brazos.py                      # cada motor, de a uno: +delta, -delta y vuelta a su posición inicial
    python3 test_brazos.py --juntos             # los 6 a la vez
    python3 test_brazos.py --juntos --ciclos 3  # repetir 3 veces
    python3 test_brazos.py --ids 4 5 6          # solo el brazo derecho
    python3 test_brazos.py --ids 3 --goto 2048  # llevar un motor a una posición absoluta (2048 = centro)
    python3 test_brazos.py --leer               # solo leer el estado, sin mover nada

En la Raspi el SDK viene con ROS: antes, `source /opt/ros/jazzy/setup.bash`.
Por defecto usa /dev/jonas_u2d2 (Raspi). En la laptop: --port /dev/ttyUSB0
Al terminar (o con Ctrl+C) apaga el torque de todos los motores.

Atención: verifica que cada articulación tenga espacio libre para moverse ±(delta * 360/4096) grados.
"""
import argparse
import time

from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler

# Tabla de control MX-28(2.0)
ADDR_TORQUE, ADDR_HW_ERROR = 64, 70
ADDR_PROFILE_ACC, ADDR_PROFILE_VEL = 108, 112
ADDR_GOAL, ADDR_MOVING, ADDR_PRESENT = 116, 122, 132
ADDR_VOLTAGE, ADDR_TEMP = 144, 146
RPM_PER_UNIT = 0.229          # Profile Velocity
MODEL_MX28_P2 = 30
POS_MIN, POS_MAX = 0, 4095    # rango del modo Position (3)
TOL = 8                       # pasos (~0.7°) para considerar que llegó


def deg(p):
    return p * 360 / 4096


class Bus:
    def __init__(self, port, baud, ids):
        self.ids = ids
        self.port = PortHandler(port)
        self.ph = PacketHandler(2.0)
        if not (self.port.openPort() and self.port.setBaudRate(baud)):
            raise SystemExit(f"No se pudo abrir {port} a {baud} bps (¿Wizard abierto?)")
        self.reader = GroupSyncRead(self.port, self.ph, ADDR_PRESENT, 4)
        for i in ids:
            self.reader.addParam(i)

    def check(self, result, error, what):
        if result != COMM_SUCCESS:
            raise RuntimeError(f"{what}: {self.ph.getTxRxResult(result)}")
        if error:
            raise RuntimeError(f"{what}: {self.ph.getRxPacketError(error)}")

    def write(self, i, addr, size, value, what):
        fn = {1: self.ph.write1ByteTxRx, 2: self.ph.write2ByteTxRx, 4: self.ph.write4ByteTxRx}[size]
        self.check(*fn(self.port, i, addr, value), f"ID {i}: {what}")

    def read(self, i, addr, size):
        fn = {1: self.ph.read1ByteTxRx, 2: self.ph.read2ByteTxRx, 4: self.ph.read4ByteTxRx}[size]
        value, result, error = fn(self.port, i, addr)
        self.check(result, error, f"ID {i}: leer {addr}")
        return value

    def positions(self):
        result = self.reader.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"Sync Read: {self.ph.getTxRxResult(result)}")
        pos = {}
        for i in self.ids:
            if not self.reader.isAvailable(i, ADDR_PRESENT, 4):
                raise RuntimeError(f"ID {i}: sin respuesta en Sync Read")
            pos[i] = self.reader.getData(i, ADDR_PRESENT, 4)
        return pos

    def set_goals(self, goals):
        writer = GroupSyncWrite(self.port, self.ph, ADDR_GOAL, 4)
        for i, g in goals.items():
            g = max(POS_MIN, min(POS_MAX, int(g)))
            writer.addParam(i, list(g.to_bytes(4, "little")))
        result = writer.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"Sync Write: {self.ph.getTxRxResult(result)}")

    def torque(self, on):
        for i in self.ids:
            self.write(i, ADDR_TORQUE, 1, 1 if on else 0, "torque")

    def status(self, title):
        print(f"\n{title}")
        print("  ID   posición      voltaje  temp  error")
        for i in self.ids:
            p = self.read(i, ADDR_PRESENT, 4)
            v = self.read(i, ADDR_VOLTAGE, 2) / 10
            t = self.read(i, ADDR_TEMP, 1)
            e = self.read(i, ADDR_HW_ERROR, 1)
            print(f"  {i:2d}   {p:4d} ({deg(p):5.1f}°)  {v:5.1f} V  {t:3d}°C  {e}")

    def move(self, goals, timeout=15.0):
        """Envía los objetivos y espera a que todos lleguen."""
        goals = {i: max(POS_MIN, min(POS_MAX, int(g))) for i, g in goals.items()}
        self.set_goals(goals)
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.05)
            pos = self.positions()
            if all(abs(pos[i] - goals[i]) <= TOL for i in goals):
                break
        else:
            print("  Aviso: no todos llegaron a tiempo (¿algo los bloquea?)")
        txt = "  ".join(f"ID{i}:{pos[i]:4d}" for i in goals)
        print(f"  objetivo -> {txt}")
        return pos


def main():
    ap = argparse.ArgumentParser(description="Prueba de los 6 MX-28 de los brazos")
    ap.add_argument("--port", default="/dev/jonas_u2d2")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--delta", type=int, default=150, help="recorrido a cada lado en pasos (150 ≈ 13°)")
    ap.add_argument("--rpm", type=float, default=10.0, help="velocidad máxima del perfil")
    ap.add_argument("--acc", type=int, default=20, help="Profile Acceleration (×214.6 rev/min²; 0 = sin límite)")
    ap.add_argument("--juntos", action="store_true", help="mover todos a la vez")
    ap.add_argument("--ciclos", type=int, default=1)
    ap.add_argument("--goto", type=int, help="llevar los --ids a esta posición absoluta (0–4095) y terminar")
    ap.add_argument("--leer", action="store_true", help="solo leer el estado")
    args = ap.parse_args()

    bus = Bus(args.port, args.baud, args.ids)
    try:
        for i in args.ids:
            model, result, error = bus.ph.ping(bus.port, i)
            bus.check(result, error, f"ping ID {i}")
            if model != MODEL_MX28_P2:
                raise SystemExit(f"ID {i} es modelo {model}, no MX-28(2.0); no muevo nada")
        bus.status("Estado inicial:")
        if args.leer:
            return

        # Profile Velocity / Acceleration (se escriben con torque apagado) y luego torque ON
        vel = max(1, round(args.rpm / RPM_PER_UNIT))
        bus.torque(False)
        for i in args.ids:
            bus.write(i, ADDR_PROFILE_VEL, 4, vel, "profile velocity")
            bus.write(i, ADDR_PROFILE_ACC, 4, args.acc, "profile acceleration")
        start = bus.positions()
        bus.torque(True)
        print(f"\nTorque ON  (velocidad {args.rpm} rpm)")

        if args.goto is not None:
            bus.move({i: args.goto for i in args.ids})
        elif args.juntos:
            for c in range(args.ciclos):
                print(f"\nCiclo {c + 1}/{args.ciclos} (todos juntos)")
                bus.move({i: start[i] + args.delta for i in args.ids})
                bus.move({i: start[i] - args.delta for i in args.ids})
                bus.move(dict(start))
        else:
            for c in range(args.ciclos):
                for i in args.ids:
                    print(f"\nCiclo {c + 1}/{args.ciclos}, ID {i}")
                    for g in (start[i] + args.delta, start[i] - args.delta, start[i]):
                        bus.move({i: g})
        bus.status("Estado final:")
    except KeyboardInterrupt:
        print("\nInterrumpido.")
    finally:
        try:
            bus.torque(False)
            print("Torque OFF en todos.")
        except Exception as e:  # noqa: BLE001
            print("Aviso: no se pudo apagar el torque de algún motor:", e)
        bus.port.closePort()


if __name__ == "__main__":
    main()
