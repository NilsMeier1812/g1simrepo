#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_store.py — einfache, dateibasierte Ablage fuer gespeicherte Oberkoerper-/
Armposen (siehe g1pilot/SCENE_BRIDGE.md Abschnitt 9: Positionsspeicher).

Bewusst lokal-first (JSON-Datei, kein Datenbank-Server): offline-faehig, keine
Netzabhaengigkeit beim Pose-Recall am echten Roboter. Gespeichert wird NUR der
Endpunkt (7-DOF-Gelenkwinkel je Arm) -- die Bahn dorthin wird beim Anfahren
JEDES MAL neu geplant (siehe arm_planner.py), weil die Startpose variiert und
sich Hindernisse/Greif-Objekte seit dem Speichern bewegt haben koennen.

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

    def save(self, name: str, q_left7, q_right7) -> None:
        if not name:
            raise ValueError("Pose-Name darf nicht leer sein.")
        data = self._load()
        data[name] = {
            "left": [float(x) for x in q_left7],
            "right": [float(x) for x in q_right7],
            "saved_at": time.time(),
        }
        self._save(data)

    def get(self, name: str):
        """-> (q_left7, q_right7) oder None, wenn der Name nicht existiert."""
        entry = self._load().get(name)
        if entry is None:
            return None
        return entry.get("left"), entry.get("right")

    def delete(self, name: str) -> bool:
        data = self._load()
        if name in data:
            del data[name]
            self._save(data)
            return True
        return False
