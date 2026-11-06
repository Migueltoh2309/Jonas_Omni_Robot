#!/usr/bin/env python3
"""Interfaz web de Jonas (modo manual): sirve la app para el celular y la conecta con ROS 2.

    ros2 run jonas_web server                 # en la Raspi → abrir http://<ip-de-la-raspi>:8080 en el celular
    python3 -m jonas_web.server --mock        # sin ROS (laptop): muestra en consola lo que se publicaría

El celular habla con este nodo por WebSocket (/ws, mensajes JSON):
    celular → robot: {"t":"cmd","vx":-1..1,"vy":-1..1,"wz":-1..1,"speed":0..1}   (~20 Hz mientras se usa el joystick)
                     {"t":"stop"}  {"t":"emotion","name":..}  {"t":"action","name":..}  {"t":"mode","name":..}
    robot → celular: {"t":"hello",...} al conectar y {"t":"state",...} 5 veces por segundo

Publica:
    /cmd_vel               geometry_msgs/Twist   x adelante, y izquierda (m/s), z giro antihorario (rad/s)
    /jonas/face/expression std_msgs/String
    /jonas/arms/action     std_msgs/String       (lo consumirá jonas_arms)
    /jonas/mode            std_msgs/String       manual | auto
Lee: /jonas/vision/faces (std_msgs/Int32), /jonas/status (std_msgs/String)

Seguridad: los comandos valen `cmd_timeout` s (0.5). Si el celular deja de mandar (soltó el joystick, se cortó el Wi-Fi,
se bloqueó la pantalla), el robot se detiene. Las velocidades se limitan a max_linear / max_angular.
"""
import argparse
import asyncio
import json
import os
import socket
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
    def __init__(self, cfg):
        self.cfg = cfg
        self.faces = 0
        self.status = "simulado (sin ROS)"
        self._last = None

    def publish_cmd(self, vx, vy, wz):
        cur = (round(vx, 2), round(vy, 2), round(wz, 2))
        if cur != self._last:
            print(f"/cmd_vel  vx={vx:+.2f} m/s  vy={vy:+.2f} m/s  wz={wz:+.2f} rad/s", flush=True)
            self._last = cur

    def publish(self, topic, text):
        print(f"{topic}  '{text}'", flush=True)

    def shutdown(self):
        pass


class RosBackend:
    def __init__(self, cfg):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from std_msgs.msg import Int32, String

        self.rclpy, self.Twist, self.String = rclpy, Twist, String
        rclpy.init()
        self.node = Node("jonas_web")
        P = self.node.declare_parameter
        cfg["port"] = P("port", cfg["port"]).value
        cfg["max_linear"] = P("max_linear", cfg["max_linear"]).value
        cfg["max_angular"] = P("max_angular", cfg["max_angular"]).value
        cfg["cmd_timeout"] = P("cmd_timeout", cfg["cmd_timeout"]).value
        self.pub_cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.pubs = {t: self.node.create_publisher(String, t, 10)
                     for t in ("/jonas/face/expression", "/jonas/arms/action", "/jonas/mode")}
        self.faces = 0
        self.status = ""
        self.node.create_subscription(Int32, "/jonas/vision/faces", lambda m: setattr(self, "faces", m.data), 10)
        self.node.create_subscription(String, "/jonas/status", lambda m: setattr(self, "status", m.data), 10)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def publish_cmd(self, vx, vy, wz):
        m = self.Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        self.pub_cmd.publish(m)

    def publish(self, topic, text):
        self.pubs[topic].publish(self.String(data=text))
        self.node.get_logger().info(f"{topic}: {text}")

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
        elif t == "emotion" and msg.get("name") in EMOTIONS:
            self.emotion = msg["name"]
            self.b.publish("/jonas/face/expression", self.emotion)
        elif t == "action" and msg.get("name") in ACTIONS:
            self.last_action = msg["name"]
            self.b.publish("/jonas/arms/action", self.last_action)
        elif t == "mode" and msg.get("name") in ("manual", "auto"):
            self.stop()
            self.mode = msg["name"]
            self.b.publish("/jonas/mode", self.mode)

    def stop(self):
        self.cmd, self.cmd_t = (0.0, 0.0, 0.0), 0.0
        self.b.publish_cmd(0.0, 0.0, 0.0)
        self.moving = False

    async def control_loop(self):
        """Publica /cmd_vel a 20 Hz mientras haya comandos frescos; si caducan, frena una vez."""
        while True:
            await asyncio.sleep(0.05)
            fresh = time.monotonic() - self.cmd_t < self.cfg["cmd_timeout"]
            if fresh and self.mode == "manual":
                self.b.publish_cmd(*self.cmd)
                self.moving = any(abs(v) > 1e-3 for v in self.cmd)
            elif self.moving or self.cmd_t:
                self.stop()

    def state(self):
        vx, vy, wz = self.cmd if self.moving else (0.0, 0.0, 0.0)
        return {"t": "state", "mode": self.mode, "emotion": self.emotion, "action": self.last_action,
                "cmd": [round(vx, 3), round(vy, 3), round(wz, 3)], "faces": self.b.faces,
                "status": self.b.status, "clients": len(self.clients)}

    async def state_loop(self):
        while True:
            await asyncio.sleep(0.2)
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

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=2.0)
        await ws.prepare(request)
        robot.clients.add(ws)
        await ws.send_str(json.dumps({"t": "hello", "emotions": EMOTIONS, "actions": ACTIONS,
                                      "max_linear": robot.cfg["max_linear"],
                                      "max_angular": robot.cfg["max_angular"], **robot.state()}))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        robot.on_message(json.loads(msg.data))
                    except (ValueError, TypeError):
                        pass
        finally:
            robot.clients.discard(ws)
            if not robot.clients:
                robot.stop()                    # se fue el último celular: frenar
        return ws

    async def start_loops(app):
        app["loops"] = [asyncio.create_task(robot.control_loop()), asyncio.create_task(robot.state_loop())]

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
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
    ap.add_argument("--mock", action="store_true", help="sin ROS: imprime lo que se publicaría")
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
