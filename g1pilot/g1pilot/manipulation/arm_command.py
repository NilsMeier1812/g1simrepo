#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_command.py — Wire-Format der Live-Pose-Schnittstelle (siehe g1pilot/docs/21_arm_api_technik.md).

BEWUSST OHNE ROS-/Pinocchio-Import: hier steckt nur das Parsen und Validieren
der eingespielten Kommandos. Dadurch ist das Format (a) an EINER Stelle
definiert -- der ROS-Eingang in arm_controller.py und die HTTP-Bruecke in
arm_api.py benutzen dieselbe Funktion, koennen also nicht auseinanderlaufen --
und (b) ohne laufenden Roboter/ROS-Graph testbar.

Fremde Projekte spielen Ziele ein, die DIREKT ausgefuehrt werden (nichts wird
gespeichert -- dafuer ist pose_store.py da). Zwei Varianten:

  type="joints"  Gelenkwinkel, 7 je Arm [rad]. Eindeutig, keine IK noetig.
  type="pose"    Kartesische Hand-Pose (Position + Orientierung). Der Sinn der
                 Schnittstelle: der Aufrufer hat NUR die Wunsch-Pose, die
                 Gelenkwinkel rechnet g1pilot per IK aus.

Validiert wird HART und mit sprechendem Grund (der Aufrufer sieht ihn im
Status/HTTP-Fehler wieder) -- ein halb verstandenes Kommando darf niemals in
eine Armbewegung muenden.
"""
import json
import math
import uuid

# Einzige Definition der speicherbaren Komponenten: der Store fuehrt sie, hier
# werden sie nur validiert. (pose_store ist wie dieses Modul ROS-frei.)
from g1pilot.manipulation.pose_store import COMPONENTS

SIDES = ("left", "right")
ARM_DOF = 7
HAND_DOF = 6

# Bewegungsarten, die ein Kommando anfordern kann.
TYPE_JOINTS = "joints"
TYPE_POSE = "pose"
COMMAND_TYPES = (TYPE_JOINTS, TYPE_POSE)

# Status-Zustaende auf /g1pilot/arm_command/status (g1pilot/docs/21_arm_api_technik.md dokumentiert sie).
ST_ACCEPTED = "accepted"     # angenommen, Planung laeuft
ST_REJECTED = "rejected"     # abgelehnt (Format, Limits, Arme nicht bereit) -- Endzustand
ST_EXECUTING = "executing"   # Bahn wird abgefahren
ST_REACHED = "reached"       # Ziel erreicht -- Endzustand
ST_FAILED = "failed"         # IK/Planung fehlgeschlagen -- Endzustand
ST_CANCELLED = "cancelled"   # abgebrochen (E-Stop, Marker, cancel) -- Endzustand
TERMINAL_STATES = (ST_REJECTED, ST_REACHED, ST_FAILED, ST_CANCELLED)

# Speichern (Positionsspeicher) laeuft ueber ein eigenes Topic und hat einen
# eigenen Erfolgs-Endzustand -- sonst dieselbe Zustands-Sprache wie oben.
ST_SAVED = "saved"
SAVE_TERMINAL_STATES = (ST_SAVED, ST_REJECTED, ST_FAILED)
# Alles, worauf ein wartender Aufrufer (arm_api) als "fertig" reagieren darf.
ALL_TERMINAL_STATES = tuple(set(TERMINAL_STATES) | set(SAVE_TERMINAL_STATES))

# Kurzformen fuer die Komponenten-Auswahl beim Speichern. 'hand' ist die
# Altform der GUI (beide Haende) und bleibt bewusst gueltig.
COMPONENT_ALIASES = {
    "arms": ("left_arm", "right_arm"),
    "hands": ("left_hand", "right_hand"),
    "hand": ("left_hand", "right_hand"),
    "all": COMPONENTS,
}


class CommandError(ValueError):
    """Ungueltiges Kommando. Die Meldung geht 1:1 an den Aufrufer zurueck."""


def new_request_id() -> str:
    """Kurze, eindeutige Vorgangs-ID (der Aufrufer kann seine eigene mitgeben)."""
    return uuid.uuid4().hex[:12]


def _finite_floats(values, n, what):
    try:
        seq = list(values)
    except TypeError:
        raise CommandError(f"{what}: erwarte eine Liste mit {n} Zahlen.")
    if len(seq) != n:
        raise CommandError(f"{what}: erwarte {n} Werte, bekam {len(seq)}.")
    out = []
    for i, v in enumerate(seq):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise CommandError(f"{what}[{i}]: keine Zahl ({v!r}).")
        f = float(v)
        if not math.isfinite(f):
            raise CommandError(f"{what}[{i}]: NaN/Inf ist nicht erlaubt.")
        out.append(f)
    return out


def _quat_from_rpy_deg(rpy_deg):
    """RPY [deg] (ZYX-Konvention, wie in den ee_offset_*_rpy_deg-Parametern)
    -> Quaternion [x, y, z, w]. Bequemlichkeit fuer Aufrufer ohne Quaternion-
    Bibliothek."""
    r, p, y = (math.radians(a) for a in rpy_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _parse_pose_target(side, spec):
    """-> {"position": [3], "quat_xyzw": [4], "frame": str}"""
    if not isinstance(spec, dict):
        raise CommandError(
            f"{side}: erwarte ein Objekt mit 'position' und 'orientation'/'rpy_deg'.")
    if "position" not in spec:
        raise CommandError(f"{side}: 'position' [x, y, z] fehlt.")
    pos = _finite_floats(spec["position"], 3, f"{side}.position")

    if "orientation" in spec and "rpy_deg" in spec:
        raise CommandError(f"{side}: 'orientation' UND 'rpy_deg' angegeben -- nur eines.")
    if "rpy_deg" in spec:
        quat = _quat_from_rpy_deg(_finite_floats(spec["rpy_deg"], 3, f"{side}.rpy_deg"))
    elif "orientation" in spec:
        quat = _finite_floats(spec["orientation"], 4, f"{side}.orientation")
        norm = math.sqrt(sum(c * c for c in quat))
        if norm < 1e-6:
            raise CommandError(f"{side}.orientation: Quaternion hat Laenge 0.")
        quat = [c / norm for c in quat]   # normieren, statt den Solver zu verbiegen
    else:
        raise CommandError(
            f"{side}: Orientierung fehlt -- 'orientation' [x, y, z, w] oder 'rpy_deg'.")

    frame = spec.get("frame", "")
    if not isinstance(frame, str):
        raise CommandError(f"{side}.frame: erwarte einen Frame-Namen als Text.")
    return {"position": pos, "quat_xyzw": quat, "frame": frame.strip()}


def parse_command(raw) -> dict:
    """Rohes Kommando (JSON-Text oder dict) -> normalisiertes Kommando-dict:

        {"id": str, "type": "joints"|"pose", "sides": [...],
         "targets": {side: [7 rad] | {"position","quat_xyzw","frame"}},
         "hands": {side: [6]}, "apply_ee_offset": bool}

    Wirft CommandError mit einer Begruendung, die dem Aufrufer gezeigt wird.
    Prueft NUR das Format (Struktur, Anzahl, Zahlen) -- Gelenklimits, Kollision,
    Reichweite und ob die Arme ueberhaupt bereit sind, prueft der
    arm_controller, weil dafuer Robotermodell und Live-Zustand noetig sind."""
    if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("utf-8", errors="replace")
    if isinstance(raw, str):
        text = raw.strip()
        if not text:
            raise CommandError("Leeres Kommando.")
        try:
            raw = json.loads(text)
        except ValueError as e:
            raise CommandError(f"Kein gueltiges JSON: {e}")
    if not isinstance(raw, dict):
        raise CommandError("Erwarte ein JSON-Objekt.")

    cmd_type = raw.get("type", TYPE_JOINTS if "left" in raw or "right" in raw else None)
    if cmd_type not in COMMAND_TYPES:
        raise CommandError(
            f"'type' fehlt oder unbekannt -- erwarte {' oder '.join(COMMAND_TYPES)}.")

    req_id = raw.get("id") or new_request_id()
    if not isinstance(req_id, str) or len(req_id) > 64:
        raise CommandError("'id': erwarte Text mit maximal 64 Zeichen.")

    targets = {}
    for side in SIDES:
        if raw.get(side) is None:
            continue
        if cmd_type == TYPE_JOINTS:
            targets[side] = _finite_floats(raw[side], ARM_DOF, f"{side} (Gelenkwinkel)")
        else:
            targets[side] = _parse_pose_target(side, raw[side])
    if not targets:
        raise CommandError("Kein Ziel angegeben -- erwarte 'left' und/oder 'right'.")

    hands = {}
    raw_hands = raw.get("hands")
    if raw_hands is not None:
        if not isinstance(raw_hands, dict):
            raise CommandError("'hands': erwarte ein Objekt mit 'left'/'right'.")
        for side in SIDES:
            if raw_hands.get(side) is None:
                continue
            hands[side] = _finite_floats(
                raw_hands[side], HAND_DOF, f"hands.{side} (Fingerwinkel)")

    offset = raw.get("apply_ee_offset", True)
    if not isinstance(offset, bool):
        raise CommandError("'apply_ee_offset': erwarte true oder false.")

    return {
        "id": req_id,
        "type": cmd_type,
        # Feste Reihenfolge links-dann-rechts = Spaltenreihenfolge im
        # gemeinsamen Gelenkvektor der Planung (siehe arm_planner).
        "sides": [s for s in SIDES if s in targets],
        "targets": targets,
        "hands": hands,
        "apply_ee_offset": offset,
    }


def parse_save_command(raw) -> dict:
    """Rohes SPEICHER-Kommando (JSON-Text oder dict) -> normalisiert:

        {"id": str, "name": str, "category": str, "components": [...]}

    Anders als parse_command() faehrt hier nichts -- die aktuelle Stellung wird
    in die Pose-Datei geschrieben (siehe pose_store.py). `category` leer =
    Default-Kategorie (der Store setzt sie ein). `components` erlaubt die vier
    Schluessel aus COMPONENTS und die Kurzformen aus COMPONENT_ALIASES; fehlt
    das Feld, werden BEIDE ARME gespeichert (wie die Altform der GUI).

    Rueckwaerts-kompatibel: ein reiner Text (kein JSON) gilt als Pose-Name."""
    if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("utf-8", errors="replace")
    if isinstance(raw, str):
        text = raw.strip()
        if not text:
            raise CommandError("Leeres Kommando -- 'name' fehlt.")
        if text.startswith("["):
            # Sieht nach JSON aus, ist aber kein Objekt -> nicht stillschweigend
            # als Pose-NAME durchgehen lassen.
            raise CommandError("Erwarte ein JSON-Objekt oder einen Pose-Namen.")
        if not text.startswith("{"):
            # Altform: nur der Name auf dem Topic -> beide Arme.
            return {"id": "", "name": text, "category": "",
                    "components": ["left_arm", "right_arm"]}
        try:
            raw = json.loads(text)
        except ValueError as e:
            raise CommandError(f"Kein gueltiges JSON: {e}")
    if not isinstance(raw, dict):
        raise CommandError("Erwarte ein JSON-Objekt.")

    name = raw.get("name")
    if not isinstance(name, str) or not name.strip():
        raise CommandError("'name' fehlt -- unter welchem Namen soll gespeichert werden?")
    name = name.strip()
    if len(name) > 128:
        raise CommandError("'name': maximal 128 Zeichen.")

    category = raw.get("category") or ""
    if not isinstance(category, str):
        raise CommandError("'category': erwarte Text (leer = Default-Kategorie).")
    if len(category.strip()) > 128:
        raise CommandError("'category': maximal 128 Zeichen.")

    req_id = raw.get("id") or ""
    if not isinstance(req_id, str) or len(req_id) > 64:
        raise CommandError("'id': erwarte Text mit maximal 64 Zeichen.")

    raw_comps = raw.get("components")
    if raw_comps is None:
        comps = ["left_arm", "right_arm"]
    else:
        if isinstance(raw_comps, str) or not isinstance(raw_comps, (list, tuple)):
            raise CommandError(
                "'components': erwarte eine Liste, z.B. [\"arms\", \"hands\"].")
        chosen = []
        for item in raw_comps:
            if not isinstance(item, str):
                raise CommandError(f"'components': {item!r} ist kein Text.")
            key = item.strip().lower()
            if key in COMPONENT_ALIASES:
                chosen.extend(COMPONENT_ALIASES[key])
            elif key in COMPONENTS:
                chosen.append(key)
            else:
                raise CommandError(
                    f"'components': '{item}' unbekannt -- erlaubt sind "
                    f"{', '.join(COMPONENTS)} sowie "
                    f"{', '.join(sorted(COMPONENT_ALIASES))}.")
        # Reihenfolge wie COMPONENTS, ohne Duplikate (arms + left_arm ist ok).
        comps = [c for c in COMPONENTS if c in chosen]
        if not comps:
            raise CommandError("'components': leere Auswahl -- nichts zu speichern.")

    return {"id": req_id, "name": name, "category": category.strip(),
            "components": comps}


def save_status_message(req_id: str, state: str, *, name: str = "",
                        category: str = "", components=None, skipped=None,
                        reason: str = "") -> str:
    """JSON-Text fuer /g1pilot/pose_store/save/status. `components` = was
    TATSAECHLICH gespeichert wurde, `skipped` = angefordert, aber nicht
    verfuegbar (z.B. Haende ohne laufende inspire-Bridge)."""
    msg = {"id": req_id or "", "state": state, "source": "pose_store_save"}
    if name:
        msg["name"] = name
    if category:
        msg["category"] = category
    if components:
        msg["components"] = list(components)
    if skipped:
        msg["skipped"] = list(skipped)
    if reason:
        msg["reason"] = reason
    return json.dumps(msg, ensure_ascii=False)


def status_message(req_id: str, state: str, *, reason: str = "",
                   sides=None, detail=None, source: str = "arm_command") -> str:
    """JSON-Text fuer /g1pilot/arm_command/status. `detail` nimmt z.B. den
    IK-Restfehler je Seite auf, damit der Aufrufer sieht, WIE genau sein
    kartesisches Ziel erreichbar war."""
    msg = {"id": req_id or "", "state": state, "source": source}
    if reason:
        msg["reason"] = reason
    if sides:
        msg["sides"] = list(sides)
    if detail:
        msg["detail"] = detail
    return json.dumps(msg, ensure_ascii=False)
