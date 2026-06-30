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

from . import joint_map, tactile
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
    """Stufe 2: echte MuJoCo-Finger-Physik + Touch-Kraefte ueber den DDS-Hand-Kanal.

    Spricht mit der unitree_mujoco-Bridge (Modell g1_29dof_inspire_ftp):
      * Soll: 6 DOF/Hand (0..1000) -> 24 Finger-Gelenk-Radiant (joint_map) ->
        ``rt/inspire/cmd`` (LowCmd_, motor_cmd[k].q) -> Finger-Position-Servos.
      * Ist:  ``rt/inspire/state`` (LowState_): motor_state[0..23].q = ECHTE Winkel,
        motor_state[24..33].q = Fingerspitzen-Kraefte [N] (10 Touch-Sensoren).

    Fuellt /joint_states (echte Winkel) sowie model.angle_act / force_act / zones
    (echte Kraefte) -> die HTML-GUIs zeigen reale Werte statt 0.

    Slot-/Reihenfolgen sind kanonisch identisch zu MJCF-Generator, unitree-Bridge
    und joint_map (links dann rechts; je Hand thumb1-4, index1-2, middle1-2,
    ring1-2, little1-2; Tip-Kraefte je Hand thumb,index,middle,ring,little).
    """

    # 12 Gelenk-Suffixe je Hand in joint_map-Reihenfolge.
    _SUFFIX12 = [f"{f}_{i}" for f, n in joint_map.HAND_FINGERS for i in range(1, n + 1)]
    _TIP_ORDER = ("thumb", "index", "middle", "ring", "little")   # MJCF-Touch je Hand
    # DOF -> repraesentatives (proximales) Gelenk fuer die Winkel-Rueckrechnung.
    _DOF_REP = ["little_1", "ring_1", "middle_1", "index_1", "thumb_2", "thumb_1"]
    _DOF_TIPFINGER = ["little", "ring", "middle", "index", "thumb", "thumb"]
    _TIPZONE = {"little": "kleiner_tip", "ring": "ring_tip", "middle": "mittel_tip",
                "index": "zeige_tip", "thumb": "daumen_tip"}

    def __init__(self, publish_fn, interface: str = "lo",
                 closed_rad: Dict[str, float] | None = None, force_scale: float = 100.0):
        from g1pilot.utils.common import init_dds
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

        self.publish_fn = publish_fn
        self.closed = dict(closed_rad) if closed_rad else dict(joint_map.CLOSED_RAD)
        self.force_scale = float(force_scale)   # N -> Anzeige-Skala der GUIs
        self.joint_names = joint_map.hand_joint_names()   # 24, kanonisch
        self._state = None

        init_dds(interface)
        self.cmd_pub = ChannelPublisher("rt/inspire/cmd", LowCmd_)
        self.cmd_pub.Init()
        self.cmd_msg = unitree_hg_msg_dds__LowCmd_()
        self.state_sub = ChannelSubscriber("rt/inspire/state", LowState_)
        self.state_sub.Init(self._on_state, 10)

    def _on_state(self, msg) -> None:
        self._state = msg

    def _resolve_targets(self, model: HandModel) -> List[float]:
        """6 DOF (0..1000) aus dem Model: -1/disabled -> halten = letzter Ist-Winkel."""
        with model.lock:
            enabled = model.enabled
            angle_set = list(model.angle_set)
            angle_act = list(model.angle_act)
        out = []
        for d in range(6):
            a = angle_set[d] if (enabled and angle_set[d] is not None and angle_set[d] >= 0) \
                else angle_act[d]
            out.append(_clamp(float(a), 0.0, 1000.0))
        return out

    def _fill_model(self, model: HandModel, q12: List[float], f5: List[float]) -> None:
        radmap = {self._SUFFIX12[i]: q12[i] for i in range(len(self._SUFFIX12))}
        angle_act = []
        for d in range(6):
            suf = self._DOF_REP[d]
            closed = self.closed.get(suf, 0.0)
            rad = radmap.get(suf, 0.0)
            a = 1000.0 * (1.0 - rad / closed) if closed > 1e-6 else 1000.0
            angle_act.append(int(round(_clamp(a, 0.0, 1000.0))))
        fmap = {self._TIP_ORDER[i]: max(0.0, float(f5[i])) for i in range(len(self._TIP_ORDER))}
        force_act = [_clamp(fmap[self._DOF_TIPFINGER[d]] * self.force_scale, 0.0, 1000.0)
                     for d in range(6)]
        zones = tactile.zero_zones()
        for finger, newton in fmap.items():
            zid = self._TIPZONE[finger]
            ncells = len(zones.get(zid, []))
            if ncells:
                zones[zid] = [int(min(newton * self.force_scale, 4095))] * ncells
        with model.lock:
            model.angle_act = angle_act
            model.force_act = force_act
            model.zones = zones
            model.connected = True

    def update(self, models: Dict[str, HandModel], dt: float) -> None:
        # 1) Soll-Winkel beider Haende -> 24 Gelenk-Radiant -> rt/inspire/cmd.
        left = self._resolve_targets(models["left"])
        right = self._resolve_targets(models["right"])
        names, positions = joint_map.build_joint_state(left, right, self.closed)
        for k in range(min(24, len(positions))):
            self.cmd_msg.motor_cmd[k].q = float(positions[k])
        self.cmd_pub.Write(self.cmd_msg)

        # 2) Ist-Zustand aus der Sim -> /joint_states + Models (echte Winkel + Kraefte).
        st = self._state
        if st is not None:
            q = [float(st.motor_state[k].q) for k in range(24)]
            f = [float(st.motor_state[24 + i].q) for i in range(10)]
            self.publish_fn(self.joint_names, q)
            self._fill_model(models["left"], q[0:12], f[0:5])
            self._fill_model(models["right"], q[12:24], f[5:10])
        else:
            # Noch kein State (Sim faehrt hoch) -> Soll publizieren, RViz bleibt nicht leer.
            self.publish_fn(names, positions)
