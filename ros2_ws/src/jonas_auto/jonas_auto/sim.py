"""Simulador 2D mínimo para probar el modo automático sin el robot.

Una sala rectangular con cajas y columnas, un lidar de 360 rayos (como el RPLIDAR A1) y una "persona"
que camina despacio: si está frente a la cámara y cerca, se simula una cara detectada.

    python3 -m jonas_auto.sim --seconds 120 --png recorrido.png     # prueba sin pantalla
"""
import argparse
import math
import random

import numpy as np


class SimWorld:
    def __init__(self, seed=1):
        rnd = random.Random(seed)
        self.W, self.H = 6.0, 4.0                               # sala de 6 x 4 m
        self.boxes = [(1.6, 1.0, 2.2, 1.5), (3.8, 2.4, 4.6, 2.9), (4.6, 0.6, 5.2, 1.0)]   # (x0, y0, x1, y1)
        self.circles = [(2.8, 3.0, 0.15), (1.0, 3.2, 0.20)]       # columnas / patas de mesa (x, y, r)
        self.x, self.y, self.th = 0.8, 0.6, 0.3                 # pose del robot
        self.person = [5.0, 3.4]
        self.person_v = [rnd.uniform(-0.15, 0.15), rnd.uniform(-0.15, 0.15)]
        self.rnd = rnd
        self.beams = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        self.robot_radius = 0.22                                # cuerpo real (sin margen)
        self.collisions = 0
        self.person_collisions = 0

    # --- geometría -------------------------------------------------------------------------------
    def _segments(self):
        W, H = self.W, self.H
        segs = [(0, 0, W, 0), (W, 0, W, H), (W, H, 0, H), (0, H, 0, 0)]
        for x0, y0, x1, y1 in self.boxes:
            segs += [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
        return np.array(segs, float)

    def scan(self, noise=0.01):
        """Rayos en el marco del robot: (ángulos, distancias). Sin eco más allá de 12 m."""
        ang = self.beams
        dx, dy = np.cos(ang + self.th), np.sin(ang + self.th)
        best = np.full(ang.size, np.inf)
        s = self._segments()
        ex, ey = s[:, 2] - s[:, 0], s[:, 3] - s[:, 1]
        DX, DY = dx[:, None], dy[:, None]                         # (rayos, 1) x (segmentos)
        den = DX * ey - DY * ex
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((s[:, 0] - self.x) * ey - (s[:, 1] - self.y) * ex) / den
            u = ((s[:, 0] - self.x) * DY - (s[:, 1] - self.y) * DX) / den
        hit = (t > 0) & (u >= 0) & (u <= 1) & np.isfinite(t)
        best = np.where(hit, t, np.inf).min(axis=1)
        for cx, cy, cr in self.circles + [(self.person[0], self.person[1], 0.18)]:
            ox, oy = self.x - cx, self.y - cy
            b = ox * dx + oy * dy
            c = ox * ox + oy * oy - cr * cr
            disc = b * b - c
            t = -b - np.sqrt(np.maximum(disc, 0))
            ok = (disc >= 0) & (t > 0)
            best = np.where(ok & (t < best), t, best)
        best = best + np.random.normal(0, noise, best.size)
        best[best > 12] = np.inf
        return ang.copy(), best

    def face(self):
        """Ancho en px (imagen de 320) de la cara de la persona si la cámara la ve; 0 si no."""
        dx, dy = self.person[0] - self.x, self.person[1] - self.y
        d = math.hypot(dx, dy)
        rel = (math.atan2(dy, dx) - self.th + math.pi) % (2 * math.pi) - math.pi
        if abs(rel) < math.radians(35) and d < 3.0:
            return 45.0 / max(d, 0.3) * 1.2                    # ~54 px a 1 m
        return 0.0

    def min_clearance(self):
        a, r = self.scan(noise=0)
        return float(np.min(r) - self.robot_radius)

    # --- dinámica --------------------------------------------------------------------------------
    def step(self, vx, vy, wz, dt):
        c, s = math.cos(self.th), math.sin(self.th)
        nx = self.x + (vx * c - vy * s) * dt
        ny = self.y + (vx * s + vy * c) * dt
        ox, oy = self.x, self.y
        self.x, self.y = nx, ny
        if self.min_clearance() < 0:                          # choque: no se mueve
            dp = math.hypot(self.person[0] - nx, self.person[1] - ny) - 0.18 - self.robot_radius
            self.x, self.y = ox, oy
            self.collisions += 1
            if dp < 0.02:
                self.person_collisions += 1                  # la persona se le vino encima
        self.th = (self.th + wz * dt + math.pi) % (2 * math.pi) - math.pi
        # la persona camina, rebota en las paredes y no se choca con el robot (se aleja si está muy cerca)
        rx, ry = self.person[0] - self.x, self.person[1] - self.y
        dr = math.hypot(rx, ry)
        if dr < 0.75 and (self.person_v[0] * rx + self.person_v[1] * ry) < 0:
            sp = math.hypot(*self.person_v) or 0.1
            self.person_v = [rx / dr * sp, ry / dr * sp]
        for k, lim in ((0, self.W), (1, self.H)):
            self.person[k] += self.person_v[k] * dt
            if not 0.4 < self.person[k] < lim - 0.4:
                self.person_v[k] *= -1
        if self.rnd.random() < 0.01:
            self.person_v = [self.rnd.uniform(-0.15, 0.15), self.rnd.uniform(-0.15, 0.15)]


def main():
    from jonas_auto.behavior import AutoBehavior

    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--png", help="guardar el recorrido como imagen")
    args = ap.parse_args()

    w, b = SimWorld(), AutoBehavior()
    b.set_running(True)
    dt, t, traj, states, minc = 0.1, 0.0, [], {}, 9.0
    face_t = -9.0
    while t < args.seconds:
        a, r = w.scan()
        fs = w.face()
        if fs > 0:
            face_t = t
        o = b.update(dt, a, r, face_size=fs, face_age=t - face_t)
        w.step(o.vx, o.vy, o.wz, dt)
        traj.append((w.x, w.y, o.state))
        states[o.state] = states.get(o.state, 0) + dt
        minc = min(minc, w.min_clearance())
        t += dt
    dist = sum(math.hypot(traj[i][0] - traj[i - 1][0], traj[i][1] - traj[i - 1][1]) for i in range(1, len(traj)))
    print(f"{args.seconds:.0f} s simulados: recorrió {dist:.1f} m, choques {w.collisions} "
          f"({w.person_collisions} con la persona que camina), "
          f"distancia mínima a un obstáculo {minc * 100:.0f} cm")
    print("tiempo por estado:", {k: round(v, 1) for k, v in states.items()})
    if args.png:
        _draw(w, traj, args.png)


def _draw(w, traj, path):
    import os
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    S = 120
    img = pygame.Surface((int(w.W * S), int(w.H * S)))
    img.fill((255, 255, 255))
    Y = lambda y: int((w.H - y) * S)  # noqa: E731
    for x0, y0, x1, y1 in w.boxes:
        pygame.draw.rect(img, (40, 40, 40), (int(x0 * S), Y(y1), int((x1 - x0) * S), int((y1 - y0) * S)))
    for cx, cy, cr in w.circles:
        pygame.draw.circle(img, (40, 40, 40), (int(cx * S), Y(cy)), int(cr * S))
    col = {"EXPLORANDO": (51, 185, 234), "ESQUIVANDO": (255, 160, 0), "INTERACTUANDO": (34, 181, 115),
           "BLOQUEADO": (229, 51, 42)}
    for i in range(1, len(traj)):
        pygame.draw.line(img, col.get(traj[i][2], (150, 150, 150)), (int(traj[i - 1][0] * S), Y(traj[i - 1][1])),
                         (int(traj[i][0] * S), Y(traj[i][1])), 3)
    pygame.draw.rect(img, (0, 0, 0), img.get_rect(), 3)
    pygame.image.save(img, path)


if __name__ == "__main__":
    main()
