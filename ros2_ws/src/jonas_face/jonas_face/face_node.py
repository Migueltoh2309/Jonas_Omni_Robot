#!/usr/bin/env python3
"""Nodo ROS 2 de la cara de Jonas: muestra los ojos en pantalla completa (HDMI de la Raspi).

Tópicos:
    /jonas/face/expression  std_msgs/String       neutral | feliz | triste | enojado | preocupado |
                                                  mareado | aburrido | sorprendido (también en inglés)
    /jonas/face/look        geometry_msgs/Point   x, y en [-1, 1] (x: derecha de la pantalla, y: arriba).
                                                  Pensado para el seguimiento de caras con la cámara;
                                                  si deja de llegar 1.5 s, vuelve a mirar al azar.

Parámetros: fullscreen (bool, true)
Las teclas 1-8 también cambian la expresión (útil con un teclado conectado a la Raspi).
"""
import threading

import pygame
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String

from jonas_face.eyes import FPS, Face, handle_key, open_window


class FaceNode(Node):
    def __init__(self, face):
        super().__init__("jonas_face")
        self.face = face
        self.declare_parameter("fullscreen", True)
        self.create_subscription(String, "/jonas/face/expression", self.on_expression, 10)
        self.create_subscription(Point, "/jonas/face/look", self.on_look, 10)

    def on_expression(self, msg):
        if self.face.set_expression(msg.data):
            self.get_logger().info(f"expresión: {self.face.expression}")
        else:
            self.get_logger().warn(f"expresión desconocida: '{msg.data}'")

    def on_look(self, msg):
        self.face.look_at(msg.x, msg.y)


def main():
    rclpy.init()
    face = Face()
    node = FaceNode(face)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    screen = open_window(node.get_parameter("fullscreen").value)
    clock = pygame.time.Clock()
    running = True
    try:
        while running and rclpy.ok():
            dt = clock.tick(FPS) / 1000
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    running = handle_key(face, ev.key)
            face.update(dt)
            face.draw(screen)
            pygame.display.flip()
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
