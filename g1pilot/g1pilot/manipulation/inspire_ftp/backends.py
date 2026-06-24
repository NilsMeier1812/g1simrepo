#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
backends.py — Austauschbare "Hardware"-Schicht
==============================================
Hier sitzt genau die Grenze, an der frueher die Modbus-TCP-Aufrufe an die echte
Hand standen. Ein Backend bekommt die Soll-Werte (im HandModel) und fuellt die
Ist-Werte (angle_act/force_act/zones). WIE es das tut, ist der Unterschied
zwischen den geplanten Stufen:

  * Stufe 1  ``SimJointStateBackend`` (aktiv)
       Aktuiert die Sim ueber /joint_states: die URDF-Finger-Gelenke folgen den
       Soll-Winkeln -> RViz zeigt die Bewegung. Kraft/Taktil = 0.

  * Stufe 2  ``MujocoContactBackend`` (vorbereitet, noch nicht implementiert)
       Wuerde die Finger als echte MuJoCo-Aktuatoren ansteuern (Kollisionen,
       Greifen) und aus den MuJoCo-Kontaktkraeften synthetische Taktil-/Kraft-
       werte ableiten.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Callable, Dict, List

from . import joint_map
from .model import HandModel


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class HandBackend(ABC):
    """Gemeinsame Schnittstelle aller Backends."""

    @abstractmethod
    def update(self, models: Dict[str, HandModel], dt: float) -> None:
        """Soll-Werte aus den Models lesen, Ist-Werte zurueckschreiben, Sim aktuieren."""

    def shutdown(self) -> None:  # optional
        pass


class SimJointStateBackend(HandBackend):
    """Stufe 1: bewegt die URDF-Finger-Gelenke ueber /joint_states (RViz).

    Es gibt in der Sim keine Physik fuer die Finger -> die Ist-Winkel werden aus
    den Soll-Winkeln "nachgefuehrt" (geschwindigkeitsbegrenzt, damit die GUI eine
    plausible Bewegung zeigt). Kraft- und Taktilwerte bleiben 0.

    publish_fn(names, positions): vom Node bereitgestellter /joint_states-Publisher.
    speed_scale: Einheiten/s = speed(0..1000) * speed_scale. 2.5 -> volle 1000er-
                 Spanne in ~0.4 s bei Vollgas.
    """

    def __init__(
        self,
        publish_fn: Callable[[List[str], List[float]], None],
        closed_rad: Dict[str, float] | None = None,
        speed_scale: float = 2.5,
    ):
        self.publish_fn = publish_fn
        self.closed_rad = closed_rad
        self.speed_scale = speed_scale
        self._cur: Dict[str, List[float]] = {"left": [1000.0] * 6, "right": [1000.0] * 6}
        self._target: Dict[str, List[float]] = {"left": [1000.0] * 6, "right": [1000.0] * 6}

    def update(self, models: Dict[str, HandModel], dt: float) -> None:
        for side, model in models.items():
            with model.lock:
                enabled = model.enabled
                speed = model.speed_set[0] if model.speed_set else 500
                angle_set = list(model.angle_set)

            # Soll-Winkel uebernehmen (nur bei enabled; -1 = halten).
            if enabled:
                for i, a in enumerate(angle_set):
                    if a is not None and a >= 0:
                        self._target[side][i] = _clamp(float(a), 0.0, 1000.0)

            # Ist-Winkel geschwindigkeitsbegrenzt nachfuehren.
            step = max(1.0, float(speed)) * self.speed_scale * max(dt, 0.0)
            cur, tgt = self._cur[side], self._target[side]
            for i in range(6):
                d = tgt[i] - cur[i]
                if abs(d) <= step:
                    cur[i] = tgt[i]
                else:
                    cur[i] += step if d > 0 else -step

            with model.lock:
                model.angle_act = [int(round(v)) for v in cur]
                model.force_act = [0.0] * 6          # keine Kraftsensorik in der Sim
                model.connected = True               # Sim ist immer "verbunden"
                # model.zones bleiben Null (tactile.zero_zones()).

        # /joint_states fuer beide Haende publizieren (24 Finger-Gelenke).
        names, positions = joint_map.build_joint_state(
            self._cur["left"], self._cur["right"], self.closed_rad
        )
        self.publish_fn(names, positions)


class MujocoContactBackend(HandBackend):
    """Stufe 2 (Platzhalter): echte MuJoCo-Finger-Physik + gefakte Kontaktdaten.

    Geplanter Aufbau, wenn die Inspire-Finger als Aktuatoren ins MuJoCo-Scene-XML
    aufgenommen werden:
      * update(): Soll-Winkel -> Finger-Aktuator-Sollwerte an MuJoCo senden
        (z.B. via DDS-Hand-Kanal oder direkt im unitree_mujoco-Bridge-Prozess).
      * Ist-Winkel aus dem MuJoCo-Zustand lesen.
      * Aus den MuJoCo-Kontaktkraeften pro Zone synthetische Taktil-/Kraftwerte
        ableiten und in model.zones / model.force_act schreiben.

    Bewusst noch nicht implementiert — der Sim-Backend deckt Stufe 1 ab.
    """

    def update(self, models: Dict[str, HandModel], dt: float) -> None:  # pragma: no cover
        raise NotImplementedError(
            "MujocoContactBackend ist Stufe 2 und noch nicht implementiert. "
            "Aktuell SimJointStateBackend nutzen."
        )
