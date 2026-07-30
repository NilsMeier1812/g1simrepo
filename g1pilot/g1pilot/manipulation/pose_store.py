#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_store.py — einfache, dateibasierte Ablage fuer gespeicherte Oberkoerper-/
Armposen (siehe g1pilot/SCENE_BRIDGE.md Abschnitt 9: Positionsspeicher).

Bewusst lokal-first (JSON-Datei, kein Datenbank-Server): offline-faehig, keine
Netzabhaengigkeit beim Pose-Recall am echten Roboter. Gespeichert wird NUR der
Endpunkt -- die Bahn dorthin wird beim Anfahren JEDES MAL neu geplant (siehe
arm_planner.py), weil die Startpose variiert und sich Hindernisse/Greif-Objekte
seit dem Speichern bewegt haben koennen.

KOMPONENTENWEISE: Eine Pose kann eine beliebige Teilmenge speichern --
linker Arm (7 DOF), rechter Arm (7 DOF), linke Hand (6 Inspire-DOF), rechte
Hand (6 DOF). Beim Speichern waehlt die GUI aus, was mitgenommen wird; beim
Anfahren werden genau die gespeicherten Komponenten wiederhergestellt (die
Arme geplant, die Haende direkt kommandiert).

Wird von ZWEI Prozessen genutzt (arm_controller.py speichert/laedt, die
Streamdeck-GUI in teleoperation/ui_interface.py listet zum Anzeigen) -- daher
atomares Schreiben (write-tmp-then-rename), damit ein gleichzeitiges Lesen nie
eine halb geschriebene Datei sieht.
"""
import json
import os
import time
from pathlib import Path

DEFAULT_PATH = os.path.expanduser("~/.g1pilot/arm_poses.json")

# Speicherbare Komponenten (Reihenfolge = Anzeige-/Iterationsreihenfolge).
COMPONENTS = ("left_arm", "right_arm", "left_hand", "right_hand")


class PoseStore:
    def __init__(self, path=None):
        self.path = Path(path or DEFAULT_PATH)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def _load(self) -> dict:
        if not self.path.is_file():
            return {}
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def _save(self, data: dict) -> None:
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
        os.replace(tmp, self.path)   # atomar auf POSIX -- kein halb geschriebener Zustand sichtbar

    def list_names(self) -> list:
        return sorted(self._load().keys())

    def save(self, name: str, *, left_arm=None, right_arm=None,
             left_hand=None, right_hand=None) -> None:
        """Speichert genau die uebergebenen (nicht-None) Komponenten unter
        `name`. Mindestens eine muss gesetzt sein."""
        if not name:
            raise ValueError("Pose-Name darf nicht leer sein.")
        entry = {}
        if left_arm is not None:
            entry["left_arm"] = [float(x) for x in left_arm]
        if right_arm is not None:
            entry["right_arm"] = [float(x) for x in right_arm]
        if left_hand is not None:
            entry["left_hand"] = [float(x) for x in left_hand]
        if right_hand is not None:
            entry["right_hand"] = [float(x) for x in right_hand]
        if not entry:
            raise ValueError("Nichts zum Speichern ausgewaehlt.")
        entry["saved_at"] = time.time()
        data = self._load()
        data[name] = entry
        self._save(data)

    def get(self, name: str):
        """-> dict mit den VORHANDENEN Komponenten-Schluesseln (Teilmenge von
        COMPONENTS) oder None, wenn der Name nicht existiert. Legacy-Eintraege
        (alte Zwei-Arm-Form mit 'left'/'right') werden transparent auf
        'left_arm'/'right_arm' abgebildet."""
        entry = self._load().get(name)
        if entry is None:
            return None
        out = {}
        # Legacy-Kompatibilitaet: fruehere Version speicherte immer beide Arme
        # unter 'left'/'right'.
        if "left_arm" in entry:
            out["left_arm"] = entry["left_arm"]
        elif "left" in entry:
            out["left_arm"] = entry["left"]
        if "right_arm" in entry:
            out["right_arm"] = entry["right_arm"]
        elif "right" in entry:
            out["right_arm"] = entry["right"]
        for k in ("left_hand", "right_hand"):
            if k in entry:
                out[k] = entry[k]
        return out

    def components(self, name: str):
        """-> Liste der in `name` gespeicherten Komponenten (fuer die GUI-
        Anzeige), oder None wenn nicht vorhanden."""
        got = self.get(name)
        if got is None:
            return None
        return [c for c in COMPONENTS if c in got]

    def delete(self, name: str) -> bool:
        data = self._load()
        if name in data:
            del data[name]
            self._save(data)
            return True
        return False
