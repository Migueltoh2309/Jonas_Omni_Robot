#!/usr/bin/env python3
"""Modo automático de Jonas (nivel 1): explora, esquiva con el lidar e interactúa con personas.

Suscribe:
    /scan                  sensor_msgs/LaserScan    RPLIDAR
    /jonas/face/look       geometry_msgs/Point      de jonas_vision (z = ancho de la cara en px)
    /jonas/mode            std_msgs/String          "manual" | "auto"  (solo actúa en "auto")
    /jonas/auto/cmd        std_msgs/String          JSON: {"run": true/false, "max_speed": 0.15}
Publica:
    /cmd_vel               geometry_msgs/Twist      solo en modo auto y en marcha
    /jonas/face/expression std_msgs/String          la emoción del momento (al cambiar)
    /jonas/arms/action     std_msgs/String          "saludar" al encontrarse con alguien
    /jonas/auto/state      std_msgs/String          JSON con estado, obstáculo más cercano, rumbo... (5 Hz)

Parámetros: lidar_yaw (rad, cuánto está girado el 0° del lidar respecto al frente del robot),
lidar_flip (true si el lidar está montado al revés), max_speed, robot_radius, stop_dist, slow_dist, greet.
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from jonas_auto.behavior import PAUSA, AutoBehavior, Params


class AutoNode(Node):
    def __init__(self):
        super().__init__("jonas_auto")
        P = self.declare_parameter
        self.lidar_yaw = P("lidar_yaw", 0.0).value
        self.lidar_flip = P("lidar_flip", False).value
        prm = Params()
        for name in ("max_speed", "robot_radius", "stop_dist", "slow_dist", "greet"):
            setattr(prm, name, P(name, getattr(prm, name)).value)
        self.b = AutoBehavior(prm)

        self.mode = "manual"
        self.scan = None
        self.scan_t = 0.0
        self.face_size, self.face_t = 0.0, -1e9
        self.last_emotion = ""
        self.was_moving = False
        self.out = None

        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Point, "/jonas/face/look", self.on_look, 10)
        self.create_subscription(String, "/jonas/mode", self.on_mode, 10)
        self.create_subscription(String, "/jonas/auto/cmd", self.on_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_emo = self.create_publisher(String, "/jonas/face/expression", 10)
        self.pub_act = self.create_publisher(String, "/jonas/arms/action", 10)
        self.pub_state = self.create_publisher(String, "/jonas/auto/state", 10)
        self.dt = 0.1
        self.create_timer(self.dt, self.tick)
        self.create_timer(0.2, self.publish_state)
        self.get_logger().info("modo automático listo (esperando /jonas/mode = auto y la orden de iniciar)")

    # --- entradas --------------------------------------------------------------------------------
    def on_scan(self, m):
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        if self.lidar_flip:
            a = -a
        a = a + self.lidar_yaw
        r = np.asarray(m.ranges, float)
        r[(r < m.range_min) | (r > m.range_max)] = np.inf
        self.scan, self.scan_t = (a, r), time.monotonic()

    def on_look(self, m):
        self.face_size, self.face_t = m.z, time.monotonic()

    def on_mode(self, m):
        if m.data != self.mode:
            self.mode = m.data
            self.b.set_running(False)                 # al cambiar de modo, siempre en pausa
            self.stop()
            self.get_logger().info(f"modo: {self.mode}")

    def on_cmd(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            return
        if "max_speed" in c:
            self.b.p.max_speed = max(0.03, min(0.30, float(c["max_speed"])))
        if "run" in c and self.mode == "auto":
            self.b.set_running(bool(c["run"]))
            self.get_logger().info("INICIAR" if c["run"] else "PAUSA")
            if not c["run"]:
                self.stop()

    # --- ciclo -----------------------------------------------------------------------------------
    def stop(self):
        self.pub_cmd.publish(Twist())
        self.was_moving = False

    def tick(self):
        if self.mode != "auto":
            return
        now = time.monotonic()
        if self.scan is None or now - self.scan_t > 0.5:  # sin lidar no se mueve
            if self.was_moving:
                self.stop()
            self.out = None
            return
        a, r = self.scan
        o = self.b.update(self.dt, a, r, face_size=self.face_size, face_age=now - self.face_t)
        self.out = o
        if self.b.running:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = o.vx, o.vy, o.wz
            self.pub_cmd.publish(t)
            self.was_moving = True
            if o.emotion != self.last_emotion:
                self.pub_emo.publish(String(data=o.emotion))
                self.last_emotion = o.emotion
            if o.action:
                self.pub_act.publish(String(data=o.action))
        elif self.was_moving:
            self.stop()

    def scan_bins(self, n=72):
        """Resumen del lidar para la app: distancia mínima en n sectores (marco del robot, desde -180°)."""
        if self.scan is None or time.monotonic() - self.scan_t > 1.0:
            return None
        a, r = self.scan
        idx = (((a + math.pi) % (2 * math.pi)) / (2 * math.pi) * n).astype(int) % n
        out = np.full(n, np.inf)
        np.minimum.at(out, idx, np.where(r > self.b.p.lidar_min, r, np.inf))
        return [round(float(v), 2) if math.isfinite(v) else None for v in out]

    def publish_state(self):
        o = self.out
        st = {"scan": self.scan_bins(), "robot_radius": self.b.p.robot_radius, "stop_dist": self.b.p.stop_dist,
              "mode": self.mode, "running": self.b.running, "max_speed": self.b.p.max_speed,
              "state": o.state if o else ("SIN LIDAR" if self.mode == "auto" else PAUSA),
              "nearest": None if not o or not math.isfinite(o.nearest) else round(o.nearest, 2),
              "heading": round(o.heading, 2) if o else 0.0, "free": round(o.free, 2) if o else 0.0,
              "emotion": o.emotion if o else "", "cmd": [round(o.vx, 2), round(o.vy, 2), round(o.wz, 2)] if o else [0, 0, 0]}
        self.pub_state.publish(String(data=json.dumps(st)))


def main():
    rclpy.init()
    node = AutoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
