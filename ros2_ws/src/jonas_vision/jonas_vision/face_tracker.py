#!/usr/bin/env python3
"""Nodo de seguimiento de caras de Jonas: cámara USB + YuNet → hacia dónde mirar.

Publica:
    /jonas/face/look       geometry_msgs/Point   x, y en [-1, 1] (x: derecha de la pantalla, y: arriba);
                                                 solo mientras sigue a alguien (la cara vuelve a mirar al azar sola)
    /jonas/vision/faces    std_msgs/Int32        cuántas caras ve (1 Hz y cuando cambia)
    /jonas/vision/snapshot sensor_msgs/CompressedImage  foto JPEG 320x240 con las caras marcadas, 1/s, solo si alguien la pide

Con mucha gente: sigue a la cara más grande (la más cercana), se queda con esa persona aunque aparezcan otras,
la suelta si desaparece más de `lost_timeout` s, y cada `switch_min`–`switch_max` s mira a otra persona.

Parámetros principales: device, width, height, rate, score_threshold, min_face, mirror, gain_x, gain_y, debug.
Con debug:=true guarda cada segundo /tmp/jonas_vision_debug.jpg con las detecciones dibujadas.
"""
import os
import random
import threading
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

from jonas_vision.yunet import YuNet


class FaceTracker(Node):
    def __init__(self):
        super().__init__("jonas_face_tracker")
        share = get_package_share_directory("jonas_vision")
        P = self.declare_parameter
        self.device = P("device", "/dev/video0").value
        self.width = P("width", 320).value             # resolución de detección (más chica = menos CPU)
        self.height = P("height", 240).value
        self.rate = float(P("rate", 10.0).value)       # detecciones por segundo
        model = P("model", os.path.join(share, "models", "face_detection_yunet_2023mar.onnx")).value
        self.score_threshold = P("score_threshold", 0.7).value
        self.min_face = P("min_face", 18).value        # px (en la imagen de detección): ignora caras muy lejanas
        self.mirror = P("mirror", True).value          # true: con la cámara mirando al frente, como la pantalla
        self.gain_x = P("gain_x", 1.0).value
        self.gain_y = P("gain_y", 1.0).value
        self.lost_timeout = P("lost_timeout", 1.0).value
        self.switch_min = P("switch_min", 5.0).value
        self.switch_max = P("switch_max", 10.0).value
        self.smoothing = P("smoothing", 0.5).value     # 0 = sin suavizado, 0.9 = muy suave
        self.debug = P("debug", False).value
        self.snapshot_rate = float(P("snapshot_rate", 1.0).value)   # fotos/s para la app (0 = no)

        self.detector = YuNet(model, (self.width, self.height), self.score_threshold)
        self.pub_look = self.create_publisher(Point, "/jonas/face/look", 10)
        self.pub_count = self.create_publisher(Int32, "/jonas/vision/faces", 10)
        self.pub_snap = self.create_publisher(CompressedImage, "/jonas/vision/snapshot", 1)
        self.last_snap = 0.0

        self.target = None          # (cx, cy, size) en px
        self.target_seen = 0.0
        self.next_switch = 0.0
        self.look = None            # (x, y) suavizado
        self.last_count = -1
        self.last_count_pub = 0.0
        self.last_debug = 0.0
        self.stats_t, self.stats_n, self.stats_ms = time.time(), 0, 0.0

        self.cap = self._open_camera()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # --- cámara ----------------------------------------------------------------------------------
    def _open_camera(self):
        dev = int(self.device) if str(self.device).isdigit() else self.device
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)    # la cámara entrega 640x480; se reduce para detectar
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 15)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            raise SystemExit(f"No se pudo abrir la cámara {self.device}")
        w, h = cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"cámara {self.device}: {int(w)}x{int(h)} @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps; "
                               f"detección a {self.width}x{self.height}, {self.rate:.0f} Hz")
        return cap

    def _loop(self):
        period = 1.0 / self.rate
        next_t = time.time()
        while self.running and rclpy.ok():
            ok = self.cap.grab()                  # vacía el buffer: siempre se procesa el cuadro más nuevo
            if not ok:
                self.get_logger().warn("sin imagen de la cámara", throttle_duration_sec=5)
                time.sleep(0.5)
                continue
            now = time.time()
            if now < next_t:
                continue
            next_t = max(next_t + period, now)
            ok, frame = self.cap.retrieve()
            if not ok:
                continue
            small = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
            t0 = time.time()
            faces = [f for f in self.detector.detect(small) if f[2] >= self.min_face]
            self._stats((time.time() - t0) * 1000)
            self._track(faces, now)
            if self.debug and now - self.last_debug > 1.0:
                self.last_debug = now
                self._save_debug(small, faces)
            if self.snapshot_rate > 0 and now - self.last_snap > 1.0 / self.snapshot_rate \
                    and self.pub_snap.get_subscription_count() > 0:    # solo si alguien la mira (la app)
                self.last_snap = now
                self._publish_snapshot(small, faces)

    # --- a quién mirar ---------------------------------------------------------------------------
    def _track(self, faces, now):
        dets = [(x + w / 2, y + h / 2, w) for x, y, w, h, _ in faces]

        if self.target is not None:
            tx, ty, ts = self.target
            best, best_d = None, max(ts * 0.8, 25)
            for d in dets:
                dist = ((d[0] - tx) ** 2 + (d[1] - ty) ** 2) ** 0.5
                if dist < best_d:
                    best, best_d = d, dist
            if best is not None:
                self.target, self.target_seen = best, now
                others = [d for d in dets if d is not best]
                if others and now > self.next_switch:          # de vez en cuando, mirar a otra persona
                    self.target = random.choice(sorted(others, key=lambda d: -d[2])[:3])
                    self.next_switch = now + random.uniform(self.switch_min, self.switch_max)
            elif now - self.target_seen > self.lost_timeout:
                self.target = None

        if self.target is None and dets:
            self.target = max(dets, key=lambda d: d[2])        # la cara más grande = la más cercana
            self.target_seen = now
            self.next_switch = now + random.uniform(self.switch_min, self.switch_max)

        if self.target is not None and self.target_seen == now:
            x = (self.target[0] / self.width * 2 - 1) * self.gain_x
            y = (1 - self.target[1] / self.height * 2) * self.gain_y
            if self.mirror:
                x = -x
            if self.look is None:
                self.look = (x, y)
            a = self.smoothing
            self.look = (a * self.look[0] + (1 - a) * x, a * self.look[1] + (1 - a) * y)
            self.pub_look.publish(Point(x=max(-1.0, min(1.0, self.look[0])),
                                        y=max(-1.0, min(1.0, self.look[1])), z=float(self.target[2])))
        elif self.target is None:
            self.look = None

        n = len(dets)
        if n != self.last_count or now - self.last_count_pub > 1.0:
            self.pub_count.publish(Int32(data=n))
            self.last_count, self.last_count_pub = n, now

    # --- utilidades ------------------------------------------------------------------------------
    def _stats(self, ms):
        self.stats_n += 1
        self.stats_ms += ms
        if time.time() - self.stats_t > 30:
            self.get_logger().info(f"{self.stats_n / (time.time() - self.stats_t):.1f} detecciones/s, "
                                   f"{self.stats_ms / self.stats_n:.1f} ms cada una")
            self.stats_t, self.stats_n, self.stats_ms = time.time(), 0, 0.0

    def _annotate(self, img, faces):
        img = img.copy()
        for x, y, w, h, s in faces:
            cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 1)
        if self.target is not None:
            cv2.circle(img, (int(self.target[0]), int(self.target[1])), 5, (0, 0, 255), -1)
        return img

    def _save_debug(self, img, faces):
        cv2.imwrite("/tmp/jonas_vision_debug.jpg", self._annotate(img, faces))

    def _publish_snapshot(self, img, faces):
        """JPEG chico (320x240, ~10 KB) con las caras marcadas, para la pestaña Auto de la app."""
        ok, jpg = cv2.imencode(".jpg", self._annotate(img, faces), [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            m = CompressedImage()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "camera"
            m.format = "jpeg"
            m.data = jpg.tobytes()
            self.pub_snap.publish(m)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = FaceTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
