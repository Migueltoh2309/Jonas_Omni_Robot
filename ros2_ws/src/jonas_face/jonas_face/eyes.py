#!/usr/bin/env python3
"""Cara de Jonas: dos ojos animados (pygame), sin dependencias de ROS.

Prueba sin ROS (en la laptop o en la Raspi):
    python3 -m jonas_face.eyes            # ventana; teclas 1-8 cambian la expresión, el mouse mueve la mirada
    python3 -m jonas_face.eyes --fullscreen

Teclas: 1 neutral  2 feliz  3 triste  4 enojado  5 preocupado  6 mareado  7 aburrido  8 sorprendido
        F pantalla completa   Esc/Q salir
"""
import argparse
import math
import random

import pygame
from pygame import gfxdraw

W, H = 960, 540            # en ventana; en pantalla completa se usa la resolución nativa
BG = (6, 8, 14)
FPS = 30                   # suficiente para una animación suave; 60 duplica el uso de CPU            # "visor" casi negro

# Parámetros por expresión.
#  open:  apertura del párpado superior (1 = abierto)
#  angle: inclinación del párpado superior en grados (+ = baja hacia el centro: enojado; − = sube hacia el centro: triste)
#  lower: cuánto sube el párpado inferior (ojos "^ ^" de feliz)
#  scale: tamaño del ojo   pupil: tamaño del iris   color: color del brillo
#  gaze:  sesgo de la mirada (x, y) en [-1, 1]   jitter: temblor   dizzy: espirales   bounce: rebote
EXPRESSIONS = {
    "neutral":     dict(open=1.00, angle=0,   lower=0.00, scale=1.00, pupil=1.00, color=(40, 150, 255)),
    "feliz":       dict(open=1.00, angle=0,   lower=0.55, scale=1.02, pupil=1.00, color=(70, 185, 255), bounce=1),
    "triste":      dict(open=0.76, angle=-24, lower=0.00, scale=0.94, pupil=1.12, color=(55, 105, 225), gaze=(0, -0.55)),
    "enojado":     dict(open=0.70, angle=26,  lower=0.18, scale=0.98, pupil=0.85, color=(255, 70, 45)),
    "preocupado":  dict(open=0.95, angle=-16, lower=0.08, scale=1.04, pupil=0.78, color=(80, 160, 255), jitter=1),
    "mareado":     dict(open=0.92, angle=0,   lower=0.00, scale=1.00, pupil=1.00, color=(150, 115, 255), dizzy=1),
    "aburrido":    dict(open=0.62, angle=0,   lower=0.12, scale=0.98, pupil=1.00, color=(45, 120, 215), gaze=(0.65, -0.1)),
    "sorprendido": dict(open=1.00, angle=-4,  lower=0.00, scale=1.16, pupil=0.62, color=(90, 200, 255)),
}
ALIASES = {"happy": "feliz", "sad": "triste", "angry": "enojado", "worried": "preocupado",
           "dizzy": "mareado", "bored": "aburrido", "surprised": "sorprendido"}
KEYS = ["neutral", "feliz", "triste", "enojado", "preocupado", "mareado", "aburrido", "sorprendido"]
DEFAULTS = dict(gaze=(0.0, 0.0), jitter=0.0, dizzy=0.0, bounce=0.0)


def _lerp(a, b, k):
    return a + (b - a) * k


def _shade(c, f):
    return tuple(max(0, min(255, int(v * f))) for v in c)


class Face:
    def __init__(self):
        self.expression = "neutral"
        self.p = self._target()                 # parámetros actuales (se interpolan hacia el objetivo)
        self.t = 0.0
        self.look_ext = None                    # mirada externa (cámara), (x, y) en [-1, 1]
        self.look_ext_age = 99.0
        self.gaze = [0.0, 0.0]
        self.idle_target = (0.0, 0.0)
        self.next_saccade = 1.0
        self.next_blink = 2.0
        self.blink_t = -1.0                     # < 0: sin parpadeo en curso

    # --- API -------------------------------------------------------------------------------------
    def set_expression(self, name):
        name = ALIASES.get(name.strip().lower(), name.strip().lower())
        if name in EXPRESSIONS:
            self.expression = name
            return True
        return False

    def look_at(self, x, y):
        """Mirada externa en [-1, 1] (x: derecha de la pantalla, y: arriba). Caduca a los 1.5 s."""
        self.look_ext = (max(-1.0, min(1.0, x)), max(-1.0, min(1.0, y)))
        self.look_ext_age = 0.0

    # --- animación -------------------------------------------------------------------------------
    def _target(self):
        d = dict(DEFAULTS)
        d.update(EXPRESSIONS[self.expression])
        return d

    def update(self, dt):
        self.t += dt
        tgt = self._target()
        k = 1 - math.exp(-dt * 9)               # interpolación suave (~0.3 s)
        for key, v in tgt.items():
            if isinstance(v, tuple):
                self.p[key] = tuple(_lerp(a, b, k) for a, b in zip(self.p[key], v))
            else:
                self.p[key] = _lerp(self.p[key], v, k)

        # mirada: externa (cámara) o movimientos sacádicos aleatorios
        self.look_ext_age += dt
        if self.look_ext is not None and self.look_ext_age < 1.5:
            goal = self.look_ext
        else:
            self.next_saccade -= dt
            if self.next_saccade <= 0:
                self.idle_target = (random.uniform(-0.6, 0.6), random.uniform(-0.35, 0.35))
                if random.random() < 0.35:
                    self.idle_target = (0.0, 0.0)
                self.next_saccade = random.uniform(1.2, 3.5)
            bx, by = self.p["gaze"]
            goal = (self.idle_target[0] * 0.5 + bx, self.idle_target[1] * 0.5 + by)
        kg = 1 - math.exp(-dt * 14)
        self.gaze[0] = _lerp(self.gaze[0], goal[0], kg)
        self.gaze[1] = _lerp(self.gaze[1], goal[1], kg)

        # parpadeo
        if self.blink_t >= 0:
            self.blink_t += dt
            if self.blink_t > 0.16:
                self.blink_t = -1.0
        else:
            self.next_blink -= dt
            if self.next_blink <= 0:
                self.blink_t = 0.0
                slow = self.expression == "aburrido"
                self.next_blink = random.uniform(4, 7) if slow else random.uniform(2, 5.5)
                if random.random() < 0.15:
                    self.next_blink = 0.25      # a veces doble parpadeo

    def _blink_factor(self):
        if self.blink_t < 0:
            return 1.0
        x = self.blink_t / 0.16                 # 0 → 1
        return abs(1 - 2 * x)                   # abierto → cerrado → abierto

    # --- dibujo ----------------------------------------------------------------------------------
    def draw(self, surf):
        surf.fill(BG)
        p = self.p
        breathe = 1 + 0.012 * math.sin(self.t * 1.6)
        R = H * 0.215 * p["scale"] * breathe
        bob = p["bounce"] * abs(math.sin(self.t * 4.5)) * -H * 0.025
        jx = p["jitter"] * math.sin(self.t * 53) * R * 0.025
        jy = p["jitter"] * math.cos(self.t * 47) * R * 0.02
        gx, gy = self.gaze
        for side in (-1, 1):                    # -1: ojo izquierdo (en pantalla), +1: derecho
            cx = W / 2 + side * W * 0.205 + gx * W * 0.045 + jx
            cy = H / 2 - gy * H * 0.05 + bob + jy
            if p["dizzy"] > 0.01:               # mareado: cada ojo da vueltas en sentido contrario
                a = self.t * 3.0 * side
                cx += math.cos(a) * R * 0.10 * p["dizzy"]
                cy += math.sin(a) * R * 0.10 * p["dizzy"]
            self._eye(surf, cx, cy, R, side)

    def _eye(self, surf, cx, cy, R, side):
        p = self.p
        col = p["color"]
        gx, gy = self.gaze
        icx, icy = int(cx), int(cy)

        # halo (pre-renderizado y guardado en caché: dibujarlo cada cuadro consume mucha CPU en la Raspi)
        halo = self._halo(int(R), tuple(int(c) // 4 * 4 for c in col))
        surf.blit(halo, (icx - halo.get_width() // 2, icy - halo.get_height() // 2))
        # anillo exterior brillante + fondo oscuro
        self._circle(surf, icx, icy, R, col)
        self._circle(surf, icx, icy, R * 0.88, _shade(col, 0.16))

        # iris (se desplaza con la mirada)
        px = cx + gx * R * 0.22
        py = cy - gy * R * 0.22
        if p["dizzy"] > 0.5:
            self._spiral(surf, px, py, R * 0.62, side, col)
        else:
            ir = R * 0.60 * p["pupil"]
            self._circle(surf, px, py, ir, _shade(col, 0.75))
            self._circle(surf, px, py, ir * 0.80, _shade(col, 0.42))
            self._circle(surf, px, py, ir * 0.50, _shade(col, 0.20))
            # brillos
            self._circle(surf, px - ir * 0.38, py - ir * 0.40, max(2, ir * 0.20), (235, 245, 255))
            self._circle(surf, px + ir * 0.32, py + ir * 0.30, max(1, ir * 0.08), (200, 225, 255))

        # párpado superior (polígono del color del fondo)
        openness = p["open"] * self._blink_factor()
        y0 = cy - R * 1.45 + (1 - openness) * R * 2.9    # 1: sobre el halo, 0: cubre todo el ojo
        slope = math.tan(math.radians(p["angle"])) * side * -1   # el lado interior es hacia el centro
        x1, x2 = cx - R * 1.45, cx + R * 1.45
        top = cy - R * 2
        lid = [(x1, y0 + slope * (x1 - cx)), (x2, y0 + slope * (x2 - cx)), (x2, top), (x1, top)]
        pygame.draw.polygon(surf, BG, lid)
        pygame.draw.aaline(surf, BG, lid[0], lid[1])

        # párpado inferior: círculo grande que sube desde abajo (feliz = media luna "^")
        if p["lower"] > 0.005:
            Rb = R * 1.5
            top_b = cy + R * 1.25 - p["lower"] * R * 1.35
            self._circle(surf, cx, top_b + Rb, Rb, BG)

    _halo_cache = {}

    @classmethod
    def _halo(cls, r, col):
        key = (r, col)
        h = cls._halo_cache.get(key)
        if h is None:
            if len(cls._halo_cache) > 200:
                cls._halo_cache.clear()
            size = int(r * 1.36) * 2 + 4
            h = pygame.Surface((size, size), pygame.SRCALPHA)
            c = size // 2
            for i in range(10, 0, -1):
                gfxdraw.filled_circle(h, c, c, int(r * (1 + 0.035 * i)), (*col, int(10 + 3 * (10 - i))))
            cls._halo_cache[key] = h
        return h

    @staticmethod
    def _circle(surf, x, y, r, color):
        x, y, r = int(x), int(y), max(1, int(r))
        gfxdraw.filled_circle(surf, x, y, r, color)
        gfxdraw.aacircle(surf, x, y, r, color)

    def _spiral(self, surf, x, y, r, side, col):
        pts = []
        turns = 3.0
        rot = self.t * 6 * side
        for i in range(90):
            th = i / 89 * turns * 2 * math.pi
            rr = r * i / 89
            pts.append((x + rr * math.cos(th + rot), y + rr * math.sin(th + rot)))
        pygame.draw.lines(surf, col, False, pts, max(3, int(r * 0.09)))


def open_window(fullscreen):
    """Pantalla completa: dibuja directamente a la resolución nativa (sin escalar = menos CPU).
    En ventana: 960x540 escalado."""
    global W, H
    pygame.init()
    if fullscreen:
        info = pygame.display.Info()
        W, H = info.current_w, info.current_h
        screen = pygame.display.set_mode((W, H), pygame.FULLSCREEN)
    else:
        screen = pygame.display.set_mode((W, H), pygame.SCALED | pygame.RESIZABLE, vsync=1)
    pygame.display.set_caption("Jonas")
    pygame.mouse.set_visible(not fullscreen)
    return screen


def handle_key(face, key):
    """Devuelve False si hay que salir."""
    if key in (pygame.K_ESCAPE, pygame.K_q):
        return False
    if key == pygame.K_f:
        pygame.display.toggle_fullscreen()
    idx = key - pygame.K_1
    if 0 <= idx < len(KEYS):
        face.set_expression(KEYS[idx])
    return True


def main():
    ap = argparse.ArgumentParser(description="Ojos de Jonas (sin ROS)")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--snapshots", metavar="DIR", help="guarda una imagen de cada expresión y termina")
    args = ap.parse_args()

    if args.snapshots:                          # render sin pantalla, para revisar las expresiones
        import os
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        pygame.init()
        surf = pygame.Surface((W, H))
        for name in KEYS:
            face = Face()
            face.next_blink = 99
            face.set_expression(name)
            for _ in range(90):                 # 1.5 s a 60 fps para llegar a la expresión
                face.update(1 / 60)
            face.draw(surf)
            pygame.image.save(surf, os.path.join(args.snapshots, f"{name}.png"))
        return

    screen = open_window(args.fullscreen)
    face = Face()
    clock = pygame.time.Clock()
    running = True
    while running:
        dt = clock.tick(FPS) / 1000
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                running = handle_key(face, ev.key)
            elif ev.type == pygame.MOUSEMOTION:
                mx, my = ev.pos
                face.look_at(mx / W * 2 - 1, 1 - my / H * 2)
        face.update(dt)
        face.draw(screen)
        pygame.display.flip()
    pygame.quit()


if __name__ == "__main__":
    main()
