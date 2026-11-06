#!/usr/bin/env python3
"""Interfaz web de Jonas: sirve la app para el celular y la conecta con ROS 2.

    ros2 run jonas_web server                 # en la Raspi → abrir http://<ip-de-la-raspi>:8080 en el celular
    python3 -m jonas_web.server --mock        # sin ROS (laptop): robot simulado en una sala virtual

El celular habla con este nodo por WebSocket (/ws, mensajes JSON):
    celular → robot: {"t":"cmd","vx":-1..1,"vy":-1..1,"wz":-1..1,"speed":0..1}   (~20 Hz mientras se usa el joystick)
                     {"t":"stop"}  {"t":"emotion","name":..}  {"t":"action","name":..}  {"t":"mode","name":"manual|auto"}
                     {"t":"auto","run":true|false}  {"t":"auto","max_speed":0.15}
    robot → celular: {"t":"hello",...} al conectar y {"t":"state",...} 5 veces por segundo
La foto de la cámara (modo auto) se pide aparte: GET /snapshot.jpg

Publica: /cmd_vel (solo en modo manual), /jonas/face/expression, /jonas/arms/action, /jonas/mode, /jonas/auto/cmd
Lee:     /jonas/vision/faces, /jonas/status, /jonas/auto/state, /jonas/vision/snapshot, /jonas/face/expression

Seguridad (modo manual): los comandos valen `cmd_timeout` s (0.5). Si el celular deja de mandar (soltó el joystick,
se cortó el Wi-Fi, se bloqueó la pantalla), el robot se detiene. En modo auto el robot sigue solo (con su propia
protección de obstáculos); STOP lo pone en pausa.
"""
import argparse
import asyncio
import json
import math
import os
import socket
import sys
import threading
import time

from aiohttp import WSMsgType, web

EMOTIONS = ["neutral", "feliz", "triste", "enojado", "preocupado", "mareado", "aburrido", "sorprendido"]
# Acciones de los brazos (se grabarán en jonas_arms). Por ahora solo se publican.
ACTIONS = ["saludar", "reposo", "aplaudir", "abrazar"]
STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


# --------------------------------------------------------------------------------------------------
# Backends: ROS 2 o simulado (para probar la app sin robot)
# --------------------------------------------------------------------------------------------------
class MockBackend:
    """Sin ROS: un robot simulado en una sala (jonas_auto.sim) que se mueve con el joystick o en modo auto."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.faces, self.status = 0, "simulado (sin ROS)"
        self.emotion, self.auto_state, self.snapshot = None, None, None
        self.mode, self.manual_cmd = "manual", (0.0, 0.0, 0.0)
        try:
            sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "jonas_auto"))
            from jonas_auto.behavior import AutoBehavior
            from jonas_auto.sim import SimWorld
            self.world, self.beh = SimWorld(), AutoBehavior()
        except ImportError as e:
            print("Aviso: sin simulador (", e, ")", flush=True)
            self.world = None
        self.face_t = -1e9

    def publish_cmd(self, vx, vy, wz):
        self.manual_cmd = (vx, vy, wz)

    def publish_quiet(self, topic, text):
        pass

    def publish(self, topic, text):
        print(f"{topic}  '{text}'", flush=True)
        if topic == "/jonas/mode":
            self.mode = text
            if self.world:
                self.beh.set_running(False)
        elif topic == "/jonas/face/expression":
            self.emotion = text
        elif topic == "/jonas/auto/cmd" and self.world:
            c = json.loads(text)
            if "max_speed" in c:
                self.beh.p.max_speed = float(c["max_speed"])
            if "run" in c and self.mode == "auto":
                self.beh.set_running(c["run"])

    def tick(self, dt):
        """Avanza la simulación (lo llama el servidor a 10 Hz)."""
        if not self.world:
            return
        w = self.world
        a, r = w.scan()
        fs = w.face()
        if fs > 0:
            self.face_t = time.monotonic()
        self.faces = 1 if fs > 0 else 0
        o = self.beh.update(dt, a, r, face_size=fs, face_age=time.monotonic() - self.face_t)
        if self.mode == "auto":
            cmd = (o.vx, o.vy, o.wz)
            if o.emotion != self.emotion:
                self.publish("/jonas/face/expression", o.emotion)
            if o.action:
                self.publish("/jonas/arms/action", o.action)
        else:
            cmd = self.manual_cmd
        w.step(*cmd, dt)
        bins = [math.inf] * 72
        for ang, d in zip(a, r):
            i = int(((ang + math.pi) % (2 * math.pi)) / (2 * math.pi) * 72) % 72
            bins[i] = min(bins[i], d)
        self.auto_state = {"mode": self.mode, "running": self.beh.running, "max_speed": self.beh.p.max_speed,
                           "state": o.state, "nearest": round(o.nearest, 2) if math.isfinite(o.nearest) else None,
                           "heading": round(o.heading, 2), "free": round(o.free, 2), "emotion": o.emotion,
                           "cmd": [round(v, 2) for v in cmd], "robot_radius": self.beh.p.robot_radius,
                           "stop_dist": self.beh.p.stop_dist,
                           "scan": [round(v, 2) if math.isfinite(v) else None for v in bins]}

    def shutdown(self):
        pass


class RosBackend:
    def __init__(self, cfg):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import Int32, String

        self.rclpy, self.Twist, self.String = rclpy, Twist, String
        rclpy.init()
        self.node = Node("jonas_web")
        P = self.node.declare_parameter
        for k in ("port", "max_linear", "max_angular", "cmd_timeout"):
            cfg[k] = P(k, cfg[k]).value
        self.pub_cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.pubs = {t: self.node.create_publisher(String, t, 10)
                     for t in ("/jonas/face/expression", "/jonas/arms/action", "/jonas/mode", "/jonas/auto/cmd")}
        self.faces, self.status = 0, ""
        self.emotion, self.auto_state, self.snapshot = None, None, None
        sub = self.node.create_subscription
        sub(Int32, "/jonas/vision/faces", lambda m: setattr(self, "faces", m.data), 10)
        sub(String, "/jonas/status", lambda m: setattr(self, "status", m.data), 10)
        sub(String, "/jonas/face/expression", lambda m: setattr(self, "emotion", m.data), 10)
        sub(String, "/jonas/auto/state", self._on_auto_state, 10)
        sub(CompressedImage, "/jonas/vision/snapshot", lambda m: setattr(self, "snapshot", bytes(m.data)), 1)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _on_auto_state(self, m):
        try:
            self.auto_state = json.loads(m.data)
        except ValueError:
            pass

    def publish_cmd(self, vx, vy, wz):
        m = self.Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        self.pub_cmd.publish(m)

    def publish(self, topic, text):
        self.publish_quiet(topic, text)
        self.node.get_logger().info(f"{topic}: {text}")

    def publish_quiet(self, topic, text):
        self.pubs[topic].publish(self.String(data=text))

    def shutdown(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


# --------------------------------------------------------------------------------------------------
# Lógica de la interfaz (igual con ROS o simulado)
# --------------------------------------------------------------------------------------------------
class Robot:
    def __init__(self, backend, cfg):
        self.b, self.cfg = backend, cfg
        self.mode = "manual"
        self.emotion = "neutral"
        self.last_action = ""
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = 0.0
        self.moving = False
        self.clients = set()

    def on_message(self, msg):
        t = msg.get("t")
        if t == "cmd":
            if self.mode != "manual":
                return
            speed = _clip(float(msg.get("speed", 0.5)), 0.0, 1.0)
            lin, ang = self.cfg["max_linear"] * speed, self.cfg["max_angular"] * speed
            self.cmd = (_clip(float(msg.get("vx", 0)), -1, 1) * lin,
                        _clip(float(msg.get("vy", 0)), -1, 1) * lin,
                        _clip(float(msg.get("wz", 0)), -1, 1) * ang)
            self.cmd_t = time.monotonic()
        elif t == "stop":
            self.stop()
        elif t == "emotion" and msg.get("name") in EMOTIONS and self.mode == "manual":   # en auto las decide el robot
            self.emotion = msg["name"]
            self.b.publish("/jonas/face/expression", self.emotion)
        elif t == "action" and msg.get("name") in ACTIONS:
            self.last_action = msg["name"]
            self.b.publish("/jonas/arms/action", self.last_action)
        elif t == "mode" and msg.get("name") in ("manual", "auto") and msg["name"] != self.mode:
            self.stop()
            self.mode = msg["name"]
            self.b.publish("/jonas/mode", self.mode)
        elif t == "auto" and self.mode == "auto":
            c = {}
            if "run" in msg:
                c["run"] = bool(msg["run"])
            if "max_speed" in msg:
                c["max_speed"] = _clip(float(msg["max_speed"]), 0.03, 0.30)
            if c:
                self.b.publish("/jonas/auto/cmd", json.dumps(c))

    def stop(self):
        """STOP: en manual frena; en auto pone el modo automático en pausa."""
        if self.mode == "auto":
            self.b.publish("/jonas/auto/cmd", json.dumps({"run": False}))
        self.cmd, self.cmd_t = (0.0, 0.0, 0.0), 0.0
        self.b.publish_cmd(0.0, 0.0, 0.0)
        self.moving = False

    async def control_loop(self):
        """Modo manual: publica /cmd_vel a 20 Hz mientras haya comandos frescos; si caducan, frena una vez."""
        k = 0
        while True:
            await asyncio.sleep(0.05)
            k += 1
            if k % 2 == 0 and hasattr(self.b, "tick"):
                self.b.tick(0.1)                          # simulador (solo en --mock)
            if self.mode != "manual":
                continue
            fresh = time.monotonic() - self.cmd_t < self.cfg["cmd_timeout"]
            if fresh:
                self.b.publish_cmd(*self.cmd)
                self.moving = any(abs(v) > 1e-3 for v in self.cmd)
            elif self.moving or self.cmd_t:
                self.stop()

    def state(self):
        if self.b.emotion:
            self.emotion = self.b.emotion                 # la emoción real (también la cambia el modo auto)
        vx, vy, wz = self.cmd if self.moving else (0.0, 0.0, 0.0)
        if self.mode == "auto" and self.b.auto_state:
            vx, vy, wz = self.b.auto_state.get("cmd", [0, 0, 0])
        return {"t": "state", "mode": self.mode, "emotion": self.emotion, "action": self.last_action,
                "cmd": [round(vx, 3), round(vy, 3), round(wz, 3)], "faces": self.b.faces,
                "status": self.b.status, "clients": len(self.clients),
                "auto": self.b.auto_state if self.mode == "auto" else None,
                "snapshot": self.b.snapshot is not None}

    async def state_loop(self):
        k = 0
        while True:
            await asyncio.sleep(0.2)
            k += 1
            if k % 5 == 0:                                # cada 1 s: repetir el modo (si otro nodo se reinicia, se entera)
                self.b.publish_quiet("/jonas/mode", self.mode)
            if self.clients:
                data = json.dumps(self.state())
                await asyncio.gather(*(ws.send_str(data) for ws in list(self.clients)), return_exceptions=True)


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


# --------------------------------------------------------------------------------------------------
# Servidor web
# --------------------------------------------------------------------------------------------------
def make_app(robot):
    async def index(_):
        return web.FileResponse(os.path.join(STATIC, "index.html"), headers={"Cache-Control": "no-cache"})

    async def snapshot(_):
        if robot.b.snapshot is None:
            raise web.HTTPNotFound()
        return web.Response(body=robot.b.snapshot, content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=2.0)
        await ws.prepare(request)
        robot.clients.add(ws)
        await ws.send_str(json.dumps({**robot.state(), "t": "hello", "emotions": EMOTIONS, "actions": ACTIONS,
                                      "max_linear": robot.cfg["max_linear"], "max_angular": robot.cfg["max_angular"]}))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        robot.on_message(json.loads(msg.data))
                    except (ValueError, TypeError):
                        pass
        finally:
            robot.clients.discard(ws)
            if not robot.clients and robot.mode == "manual":
                robot.stop()                    # se fue el último celular: frenar (en auto sigue solo)
        return ws

    async def start_loops(app):
        app["loops"] = [asyncio.create_task(robot.control_loop()), asyncio.create_task(robot.state_loop())]

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/snapshot.jpg", snapshot)
    app.router.add_static("/static", STATIC)
    app.on_startup.append(start_loops)
    return app


def local_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips)


def main():
    ap = argparse.ArgumentParser(description="Interfaz web de Jonas")
    ap.add_argument("--mock", action="store_true", help="sin ROS: robot simulado")
    ap.add_argument("--port", type=int, default=8080)
    args, _ = ap.parse_known_args()             # ignora --ros-args cuando se lanza con ros2 run

    cfg = dict(port=args.port, max_linear=0.30, max_angular=1.5, cmd_timeout=0.5)
    backend = MockBackend(cfg) if args.mock else RosBackend(cfg)
    robot = Robot(backend, cfg)
    for ip in local_ips():
        print(f"Interfaz de Jonas: http://{ip}:{cfg['port']}", flush=True)
    try:
        web.run_app(make_app(robot), host="0.0.0.0", port=cfg["port"], print=None)
    finally:
        backend.publish_cmd(0.0, 0.0, 0.0)
        backend.shutdown()


if __name__ == "__main__":
    main()
