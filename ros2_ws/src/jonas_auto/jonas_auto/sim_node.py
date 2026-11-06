#!/usr/bin/env python3
"""Robot simulado en ROS: reemplaza al robot real (ruedas + lidar + cámara) para probar el bringup en la laptop.

Suscribe:
    /cmd_vel               geometry_msgs/Twist     (lo mismo que recibiría jonas_base)
Publica:
    /scan                  sensor_msgs/LaserScan   lidar simulado (frame "laser"), 10 Hz
    /odom                  nav_msgs/Odometry       pose exacta del simulador + TF odom -> base_link
    /jonas/face/look       geometry_msgs/Point     cara simulada de la persona (como jonas_vision)
    /jonas/sim/world       visualization_msgs/MarkerArray   paredes, cajas y la persona (para RViz)
    TF estática base_link -> laser
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from jonas_auto.sim import SimWorld


def _yaw_quat(th):
    return 0.0, 0.0, math.sin(th / 2), math.cos(th / 2)


class SimNode(Node):
    def __init__(self):
        super().__init__("jonas_sim")
        self.w = SimWorld(seed=self.declare_parameter("seed", 1).value)
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_t = -1e9
        self.dt = 0.05
        self.n = 0
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_look = self.create_publisher(Point, "/jonas/face/look", 10)
        self.pub_world = self.create_publisher(MarkerArray, "/jonas/sim/world", 1)
        self.tf = TransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id, st.child_frame_id = "base_link", "laser"
        st.transform.translation.z = 0.25
        st.transform.rotation.w = 1.0
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(st)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info("robot simulado listo (sala 6 x 4 m)")

    def on_cmd(self, m):
        self.cmd, self.cmd_t = (m.linear.x, m.linear.y, m.angular.z), self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds * 1e-9 - self.cmd_t > 0.5:      # igual que el firmware: sin órdenes, se detiene
            self.cmd = (0.0, 0.0, 0.0)
        self.w.step(*self.cmd, self.dt)
        self.n += 1
        stamp = now.to_msg()
        w = self.w

        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = w.x, w.y
        q = _yaw_quat(w.th)
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = w.x, w.y
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z,
         o.pose.pose.orientation.w) = q
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = self.cmd
        self.pub_odom.publish(o)

        if self.n % 2 == 0:                                   # lidar y cámara a 10 Hz
            a, r = w.scan()
            s = LaserScan()
            s.header.stamp, s.header.frame_id = stamp, "laser"
            s.angle_min, s.angle_increment = float(a[0]), float(a[1] - a[0])
            s.angle_max = float(a[-1])
            s.scan_time, s.range_min, s.range_max = 0.1, 0.15, 12.0
            s.ranges = [float(v) for v in r]
            self.pub_scan.publish(s)
            fs = w.face()
            if fs > 0:
                rel = (math.atan2(w.person[1] - w.y, w.person[0] - w.x) - w.th + math.pi) % (2 * math.pi) - math.pi
                self.pub_look.publish(Point(x=max(-1.0, min(1.0, -rel / math.radians(35))), y=0.0, z=float(fs)))
        if self.n % 10 == 0:
            self.publish_world(stamp)

    def publish_world(self, stamp):
        w, ma, k = self.w, MarkerArray(), 0

        def mk(typ, x, y, sx, sy, sz, rgb):
            nonlocal k
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, "odom"
            m.ns, m.id, m.type, m.action = "sim", k, typ, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), sz / 2
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
            m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 0.8)
            ma.markers.append(m)
            k += 1

        th = 0.05
        for x, y, sx, sy in ((w.W / 2, 0, w.W, th), (w.W / 2, w.H, w.W, th), (0, w.H / 2, th, w.H), (w.W, w.H / 2, th, w.H)):
            mk(Marker.CUBE, x, y, sx, sy, 0.4, (0.6, 0.6, 0.6))
        for x0, y0, x1, y1 in w.boxes:
            mk(Marker.CUBE, (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0, 0.5, (0.25, 0.25, 0.25))
        for cx, cy, cr in w.circles:
            mk(Marker.CYLINDER, cx, cy, 2 * cr, 2 * cr, 0.7, (0.25, 0.25, 0.25))
        mk(Marker.CYLINDER, w.person[0], w.person[1], 0.36, 0.36, 1.7, (0.2, 0.71, 0.92))
        self.pub_world.publish(ma)


def main():
    rclpy.init()
    node = SimNode()
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
