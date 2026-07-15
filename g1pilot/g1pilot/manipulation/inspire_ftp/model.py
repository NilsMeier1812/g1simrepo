#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
model.py — Gemeinsamer Zustand einer Hand
=========================================
``HandModel`` haelt Soll- und Ist-Werte einer Inspire-Hand. Es ist die einzige
geteilte Datenstruktur zwischen den WebSocket-Servern (Controller/Viewer) und dem
Backend (das die Ist-Werte aus der Sim fuellt). Threadsicher via ``lock``.
"""

from __future__ import annotations
import threading
from typing import Dict, List

from . import tactile


class HandModel:
    def __init__(self, name: str, host: str):
        self.name = name
        self.host = host            # in der Sim nur Anzeige (kein echtes Modbus-Ziel)
        self.lock = threading.Lock()

        # ── Soll-Werte (vom Controller-GUI gesetzt) ──────────────────────────
        self.angle_set: List[int] = [-1] * 6     # -1 = keine Aktion (halten)
        self.force_set: List[int] = [500] * 6    # in der Sim nur Anzeige
        self.speed_set: List[int] = [500] * 6
        self.enabled: bool = False               # Hauptschalter

        # ── Ist-Werte (vom Backend gefuellt) ─────────────────────────────────
        self.angle_act: List[int] = [1000] * 6   # 0..1000, Start = offen
        self.force_act: List[float] = [0.0] * 6  # g (Finger-Kraft; Sim: aus Antriebskraft)
        self.zones: Dict[str, List[int]] = tactile.zero_zones()
        self.connected: bool = False

    # ── Setter (vom Controller-WebSocket aufgerufen) ─────────────────────────
    def set_angle(self, idx: int, val: int):
        with self.lock:
            self.angle_set[idx] = int(val)

    def set_force(self, idx: int, val: int):
        with self.lock:
            self.force_set[idx] = int(val)

    def set_speed_all(self, val: int):
        with self.lock:
            self.speed_set = [int(val)] * 6

    def set_enabled(self, enabled: bool):
        with self.lock:
            self.enabled = bool(enabled)
            if not enabled:
                # Wie im Original: bei Deaktivieren keine aktive Bewegung mehr.
                self.angle_set = [-1] * 6

    # ── Snapshots fuer die GUIs ──────────────────────────────────────────────
    def controller_state(self) -> dict:
        """Format fuer hand_controller_viewer.html (Typ 'state'/'config')."""
        with self.lock:
            return {
                "connected": self.connected,
                "host":      self.host,
                "name":      self.name,
                "angle_act": list(self.angle_act),
                "force_act": list(self.force_act),
                "angle_set": list(self.angle_set),
                "force_set": list(self.force_set),
                "speed_set": self.speed_set[0],
                "enabled":   self.enabled,
            }

    def viewer_state(self) -> dict:
        """Format fuer inspire_hand_viewer.html (Typ 'data')."""
        with self.lock:
            zones = {k: list(v) for k, v in self.zones.items()}
            raw: List[int] = []
            for z in tactile.TACTILE_ZONES:
                raw.extend(zones.get(z[0], []))
            return {
                "connected":   self.connected,
                "host":        self.host,
                "name":        self.name,
                "force":       [round(abs(f) / 100.0, 1) for f in self.force_act],
                "angle":       list(self.angle_act),
                "zones":       zones,
                "tactile_raw": raw,
            }
