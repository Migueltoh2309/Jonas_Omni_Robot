"""Comportamiento autónomo de Jonas, nivel 1 (sin mapa): explorar, esquivar con el lidar e interactuar con personas.

Este módulo NO depende de ROS: recibe el lidar y las caras, y devuelve (vx, vy, wz) + estado + emoción.
Así se puede probar en el simulador (sim.py) y usar igual en el robot (auto_node.py).

Convenciones (REP-103): x adelante, y izquierda, ángulos antihorarios, 0 = frente del robot.
"""
import math
import random
from dataclasses import dataclass, field

import numpy as np

EXPLORAR, ESQUIVAR, INTERACTUAR, PARADO, PAUSA = "EXPLORANDO", "ESQUIVANDO", "INTERACTUANDO", "BLOQUEADO", "EN PAUSA"


@dataclass
class Params:
    max_speed: float = 0.15        # m/s (se puede cambiar desde la app)
    max_turn: float = 0.8          # rad/s
    robot_radius: float = 0.25     # m, radio del robot visto desde arriba (con brazos)
    margin: float = 0.08           # m, holgura extra a cada lado del "pasillo"
    stop_dist: float = 0.25        # m, a esta distancia de algo se detiene (desde el borde del robot)
    slow_dist: float = 0.70        # m, desde aquí empieza a frenar / esquivar
    lidar_min: float = 0.12        # m, lecturas menores se ignoran (partes del propio robot)
    wander_min: float = 8.0        # s, cada cuánto cambia de rumbo al explorar
    wander_max: float = 15.0
    face_close: float = 55.0       # px (ancho de la cara en la imagen de 320 px): persona cerca
    interact_time: float = 20.0    # s máximo interactuando
    interact_lost: float = 3.0     # s sin ver la cara para despedirse
    interact_cooldown: float = 15.0
    bored_after: float = 60.0      # s sin ver a nadie → aburrido
    greet: bool = True             # al interactuar, pedir la acción "saludar" a los brazos
    directions: int = 72           # direcciones candidatas (cada 5°)


@dataclass
class Output:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    state: str = PAUSA
    emotion: str = "neutral"
    action: str = ""               # acción de brazos a ejecutar (una vez)
    nearest: float = float("inf")  # m, obstáculo más cercano (desde el borde del robot)
    heading: float = 0.0           # rad, dirección elegida para moverse (para dibujar en la app)
    free: float = 0.0              # m libres en esa dirección


@dataclass
class AutoBehavior:
    p: Params = field(default_factory=Params)
    running: bool = False

    def __post_init__(self):
        self.state = PAUSA
        self.goal = 0.0                 # rumbo deseado al explorar (en el marco del robot)
        self.t = 0.0
        self.next_wander = 0.0
        self.state_t = 0.0
        self.face_seen_t = -1e9
        self.interact_end_t = -1e9
        self.greeted = False
        self.turn_hist = []              # para detectar "mareado"
        self.dizzy_until = 0.0
        self.cand = np.linspace(-math.pi, math.pi, self.p.directions, endpoint=False)

    # ------------------------------------------------------------------------------------------
    def set_running(self, run):
        self.running = bool(run)
        self._set(EXPLORAR if run else PAUSA)
        self.goal = 0.0
        self.next_wander = self.t + random.uniform(self.p.wander_min, self.p.wander_max)

    def _set(self, st):
        if st != self.state:
            self.state, self.state_t = st, self.t

    def free_distance(self, angles, ranges):
        """Distancia libre (m) moviéndose en cada dirección candidata, considerando el ancho del robot."""
        half = self.p.robot_radius + self.p.margin
        rel = angles[None, :] - self.cand[:, None]                  # (cand, puntos)
        fwd = ranges[None, :] * np.cos(rel)
        lat = np.abs(ranges[None, :] * np.sin(rel))
        block = (fwd > 0) & (lat < half)
        dist = np.where(block, fwd - np.sqrt(np.maximum(half ** 2 - lat ** 2, 0)), np.inf)
        return np.minimum(dist.min(axis=1), 10.0)

    def _safety(self, vx, vy, a, r):
        """Última protección: quita la parte de la velocidad que va hacia cualquier punto a menos de
        stop_dist del borde del robot (puede seguir moviéndose de costado o alejándose)."""
        near = r < self.p.robot_radius + self.p.stop_dist
        for ang in a[near]:
            ux, uy = math.cos(ang), math.sin(ang)
            dot = vx * ux + vy * uy
            if dot > 0:
                vx, vy = vx - dot * ux, vy - dot * uy
        return vx, vy

    def _new_goal(self, free):
        """Nuevo rumbo al azar entre las direcciones bien libres (más probable cuanto más libre),
        evitando volver justo por donde vino: así recorre toda la sala en vez de ir y volver."""
        ok = free > self.p.slow_dist * 1.8
        back = np.abs(_wrap(self.cand - (self.goal + math.pi))) < math.radians(40)
        pick = ok & ~back
        if not pick.any():
            pick = ok
        if pick.any():
            w = np.minimum(free[pick], 3.0)
            self.goal = float(np.random.choice(self.cand[pick], p=w / w.sum()))
            self.next_wander = self.t + random.uniform(self.p.wander_min, self.p.wander_max)

    # ------------------------------------------------------------------------------------------
    def update(self, dt, angles, ranges, face_size=0.0, face_age=1e9):
        """angles/ranges: lidar en el marco del robot (rad, m; inf/nan = sin eco).
        face_size: ancho en px de la cara que se está siguiendo; face_age: s desde que se vio."""
        p = self.p
        self.t += dt
        out = Output(state=self.state)

        a = np.asarray(angles, float)
        r = np.asarray(ranges, float)
        ok = np.isfinite(r) & (r > p.lidar_min)
        a, r = a[ok], r[ok]
        out.nearest = float(r.min() - p.robot_radius) if r.size else float("inf")

        face_now = face_age < 0.5
        if face_now:
            self.face_seen_t = self.t

        if not self.running:
            self._set(PAUSA)
            out.state, out.emotion = PAUSA, "neutral"
            return out

        free = self.free_distance(a, r) if r.size else np.full(self.cand.size, 10.0)

        # --- interactuar con una persona cercana ----------------------------------------------
        if self.state != INTERACTUAR and face_now and face_size >= p.face_close \
                and self.t - self.interact_end_t > p.interact_cooldown:
            self._set(INTERACTUAR)
            self.greeted = False
        if self.state == INTERACTUAR:
            if self.t - self.face_seen_t > p.interact_lost or self.t - self.state_t > p.interact_time:
                self.interact_end_t = self.t
                self._set(EXPLORAR)
                self.goal = random.choice([-1, 1]) * random.uniform(math.pi / 2, math.pi)   # irse hacia otro lado
            else:
                out.state, out.emotion = INTERACTUAR, "feliz"
                if p.greet and not self.greeted:
                    out.action, self.greeted = "saludar", True
                return out                                    # quieto; los ojos la siguen (jonas_vision)

        # --- elegir dirección ------------------------------------------------------------------
        if self.t >= self.next_wander and self.state == EXPLORAR:
            self.goal = float(_wrap(self.goal + random.uniform(-math.pi / 2, math.pi / 2)))
            self.next_wander = self.t + random.uniform(p.wander_min, p.wander_max)

        gi = int(np.argmin(np.abs(_wrap(self.cand - self.goal))))
        if free[gi] < p.slow_dist:
            if self.state != ESQUIVAR:
                self._new_goal(free)                          # "rebotar" hacia otra zona libre, al azar
            self._set(ESQUIVAR)
        elif self.state == ESQUIVAR and free[gi] > p.slow_dist * 1.5 and self.t - self.state_t > 1.0:
            self._set(EXPLORAR)

        if self.state == ESQUIVAR:
            # la dirección más libre, prefiriendo las cercanas al rumbo actual
            score = np.minimum(free, 3.0) - 0.3 * np.abs(_wrap(self.cand - self.goal))
            bi = int(np.argmax(score))
            if free[bi] > p.slow_dist:
                self.goal = float(self.cand[bi])              # adoptar el nuevo rumbo
        else:
            # la dirección más parecida al rumbo deseado que tenga espacio suficiente
            diff = np.abs(_wrap(self.cand - self.goal))
            enough = free > p.slow_dist * 1.2
            bi = int(np.argmin(np.where(enough, diff, np.inf))) if enough.any() else int(np.argmax(free))

        heading, fdist = float(self.cand[bi]), float(free[bi])
        out.heading, out.free = heading, fdist

        if fdist < p.stop_dist:                               # sin salida: quedarse quieto
            self._set(PARADO)
            out.state, out.emotion = PARADO, "preocupado"
            out.wz = 0.5 * p.max_turn                         # girar en el sitio para buscar salida
            self.goal = float(_wrap(self.goal - out.wz * dt))
            return out
        if self.state == PARADO:
            self._set(ESQUIVAR)

        k = min(1.0, max(0.0, (fdist - p.stop_dist) / (p.slow_dist - p.stop_dist)))
        speed = p.max_speed * (0.35 + 0.65 * k)
        out.vx, out.vy = speed * math.cos(heading), speed * math.sin(heading)
        out.vx, out.vy = self._safety(out.vx, out.vy, a, r)
        # girar el cuerpo hacia donde va (la cámara y la cara miran al frente)
        out.wz = max(-p.max_turn, min(p.max_turn, 1.2 * heading)) if abs(heading) > 0.15 else 0.0
        # el rumbo deseado es fijo en el mundo: si el cuerpo gira, en el marco del robot se corre al revés
        self.goal = float(_wrap(self.goal - out.wz * dt))

        # --- emoción ---------------------------------------------------------------------------
        self.turn_hist = [(t, w) for t, w in self.turn_hist if self.t - t < 6.0] + [(self.t, abs(out.wz) * dt)]
        if sum(w for _, w in self.turn_hist) > 2 * math.pi:
            self.dizzy_until = self.t + 3.0
        if self.t < self.dizzy_until:
            out.emotion = "mareado"
        elif out.nearest < 0.35:
            out.emotion = "preocupado"
        elif self.state == ESQUIVAR:
            out.emotion = "sorprendido" if self.t - self.state_t < 1.0 else "neutral"
        elif self.t - self.face_seen_t > p.bored_after:
            out.emotion = "aburrido"
        else:
            out.emotion = "neutral"
        out.state = self.state
        return out


def _wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi
