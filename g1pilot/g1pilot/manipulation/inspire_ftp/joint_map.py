#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joint_map.py — Single Source of Truth fuer das Mapping
======================================================
Uebersetzt die 6 DOF der Inspire RH56DFTP-2 (Modbus-Winkel 0..1000) auf die
beweglichen Finger-Gelenke der g1_29dof_inspire_ftp.urdf (Radiant).

DOF-Reihenfolge (identisch zu DOF_NAMES, wie sie die HTML-GUIs erwarten):
    0 Kleiner (little)      -> little_1, little_2
    1 Ringfinger (ring)     -> ring_1,   ring_2
    2 Mittelfinger (middle) -> middle_1, middle_2
    3 Zeigefinger (index)   -> index_1,  index_2
    4 Daumen-Beugung        -> thumb_2, thumb_3, thumb_4
    5 Daumen-Rotation       -> thumb_1

Modbus-Winkelkonvention der Inspire-Hand:
    1000 = Finger offen/gestreckt   -> Gelenk 0 rad
       0 = Finger geschlossen       -> Gelenk = "closed"-Wert (siehe CLOSED_RAD)
      -1 = keine Aktion (nicht hier; -1 wird vor dem Mapping zu "halten" aufgeloest)

Die "closed"-Werte sind bewusst EXPLIZIT gesetzt (nicht stumpf upper-Limit), weil
die distalen ``*_2``-Gelenke in der URDF bis 3.14 rad zulassen (180°), was als
Vollschluss unrealistisch aussaehe. Jeder Wert wird zusaetzlich an das echte
URDF-Limit geklemmt (siehe ``load_limits_from_urdf``).
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

# ── DOF ─────────────────────────────────────────────────────────────────────
DOF_NAMES: List[str] = [
    "Kleiner", "Ringfinger", "Mittelfinger", "Zeigefinger",
    "Daumen-Beug.", "Daumen-Rot.",
]
NUM_DOF = 6

# Welche URDF-Gelenk-Suffixe haengen an welchem DOF-Index.
DOF_TO_JOINTS: List[List[str]] = [
    ["little_1", "little_2"],          # 0 Kleiner
    ["ring_1",   "ring_2"],            # 1 Ringfinger
    ["middle_1", "middle_2"],          # 2 Mittelfinger
    ["index_1",  "index_2"],           # 3 Zeigefinger
    ["thumb_2",  "thumb_3", "thumb_4"],# 4 Daumen-Beugung
    ["thumb_1"],                       # 5 Daumen-Rotation
]

# "Geschlossen"-Stellung je Gelenk in Radiant (Winkel-Eingang 0).
# An die URDF-Limits geklemmt (load_limits_from_urdf). Realistische Vollschluss-
# Werte statt der z.T. bis 3.14 rad reichenden URDF-Obergrenzen.
CLOSED_RAD: Dict[str, float] = {
    "little_1": 1.4381, "little_2": 1.40,
    "ring_1":   1.4381, "ring_2":   1.40,
    "middle_1": 1.4381, "middle_2": 1.40,
    "index_1":  1.4381, "index_2":  1.40,
    "thumb_2":  0.5864, "thumb_3":  0.50, "thumb_4": 1.20,
    "thumb_1":  1.1641,
}

# Reihenfolge der 24 beweglichen Hand-Gelenke — MUSS mit robot_state.py
# (hand_joint_names) uebereinstimmen, damit /joint_states konsistent ist.
HAND_FINGERS: Tuple[Tuple[str, int], ...] = (
    ("thumb", 4), ("index", 2), ("middle", 2), ("ring", 2), ("little", 2),
)
SIDES: Tuple[str, ...] = ("left", "right")


def hand_joint_names() -> List[str]:
    """Die 24 beweglichen Finger-Gelenke in URDF-/robot_state-Reihenfolge."""
    return [
        f"{s}_{f}_{i}_joint"
        for s in SIDES
        for f, n in HAND_FINGERS
        for i in range(1, n + 1)
    ]


def load_limits_from_urdf(urdf_path: str) -> Dict[str, float]:
    """Liest die ``upper``-Limits der Finger-Gelenke (Suffix -> rad).

    Wird genutzt, um CLOSED_RAD an die echten URDF-Grenzen zu klemmen, falls die
    URDF mal geaendert wird. Faellt bei Fehlern still auf {} zurueck.
    """
    limits: Dict[str, float] = {}
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:
        return limits
    fingers = ("thumb", "index", "middle", "ring", "little")
    for j in root.iter("joint"):
        name = j.get("name") or ""
        if j.get("type") != "revolute" or not any(f in name for f in fingers):
            continue
        lim = j.find("limit")
        if lim is None:
            continue
        # name z.B. "left_index_1_joint" -> suffix "index_1"
        suffix = name.replace("left_", "").replace("right_", "").replace("_joint", "")
        try:
            limits[suffix] = float(lim.get("upper"))
        except (TypeError, ValueError):
            continue
    return limits


def _closed_rad(clamp_to: Dict[str, float] | None = None) -> Dict[str, float]:
    """CLOSED_RAD, optional an URDF-upper-Limits geklemmt."""
    if not clamp_to:
        return dict(CLOSED_RAD)
    out = {}
    for suffix, val in CLOSED_RAD.items():
        upper = clamp_to.get(suffix)
        out[suffix] = min(val, upper) if upper is not None else val
    return out


def angles6_to_joint_rad(
    angles6: List[float],
    closed_rad: Dict[str, float] | None = None,
) -> Dict[str, float]:
    """6 DOF (0..1000) einer Hand -> {suffix: rad} fuer ihre Finger-Gelenke.

    1000 -> 0 rad (offen), 0 -> closed_rad (geschlossen), linear dazwischen.
    Werte ausserhalb 0..1000 werden geklemmt.
    """
    closed = closed_rad if closed_rad is not None else CLOSED_RAD
    out: Dict[str, float] = {}
    for dof, suffixes in enumerate(DOF_TO_JOINTS):
        a = angles6[dof] if dof < len(angles6) else 1000.0
        a = max(0.0, min(1000.0, float(a)))
        frac_closed = 1.0 - a / 1000.0          # 0=offen .. 1=geschlossen
        for suffix in suffixes:
            out[suffix] = frac_closed * closed.get(suffix, 0.0)
    return out


def build_joint_state(
    left_angles6: List[float],
    right_angles6: List[float],
    closed_rad: Dict[str, float] | None = None,
) -> Tuple[List[str], List[float]]:
    """Erzeugt (names, positions) der 24 Finger-Gelenke fuer eine /joint_states-Msg."""
    rad = {
        "left":  angles6_to_joint_rad(left_angles6,  closed_rad),
        "right": angles6_to_joint_rad(right_angles6, closed_rad),
    }
    names, positions = [], []
    for s in SIDES:
        for f, n in HAND_FINGERS:
            for i in range(1, n + 1):
                names.append(f"{s}_{f}_{i}_joint")
                positions.append(rad[s].get(f"{f}_{i}", 0.0))
    return names, positions


def default_urdf_path() -> str | None:
    """Pfad zur Inspire-FTP-URDF im installierten g1pilot-Share, falls auffindbar."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("g1pilot")
        p = os.path.join(share, "description_files", "urdf", "g1_29dof_inspire_ftp.urdf")
        return p if os.path.exists(p) else None
    except Exception:
        return None
