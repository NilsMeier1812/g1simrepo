#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_store.py — einfache, dateibasierte Ablage fuer gespeicherte Oberkoerper-/
Armposen (siehe g1pilot/docs/11_arm_manipulation_technik.md (Positionsspeicher): Positionsspeicher).

Bewusst lokal-first (JSON-Datei, kein Datenbank-Server): offline-faehig, keine
Netzabhaengigkeit beim Pose-Recall am echten Roboter. Gespeichert wird NUR der
Endpunkt -- die Bahn dorthin wird beim Anfahren JEDES MAL neu geplant (siehe
arm_planner.py), weil die Startpose variiert und sich Hindernisse/Greif-Objekte
seit dem Speichern bewegt haben koennen.

PERSISTENZ UEBER CONTAINER-NEUSTARTS: Die Datei liegt per Default IM
bind-gemounteten Repo (`<repo>/data/arm_poses.json`), NICHT unter $HOME. $HOME
liegt im Container-Overlay-Dateisystem und ist beim naechsten `docker compose
up` weg -- damit waren frueher alle Posen nach dem Schliessen der Sim verloren.
Laeuft der Code direkt auf dem Host (kein Mount), wird `~/.g1pilot/` benutzt.
Der Pfad ist per Umgebungsvariable `G1_POSE_STORE` ueberschreibbar. Eine alte
Datei unter `~/.g1pilot/arm_poses.json` wird beim ersten Zugriff automatisch in
die neue Ablage uebernommen (Migration, siehe _migrate_legacy_file).

KOMPONENTENWEISE: Eine Pose kann eine beliebige Teilmenge speichern --
linker Arm (7 DOF), rechter Arm (7 DOF), linke Hand (6 Inspire-DOF), rechte
Hand (6 DOF). Beim Speichern waehlt die GUI aus, was mitgenommen wird; beim
Anfahren werden genau die gespeicherten Komponenten wiederhergestellt (die
Arme geplant, die Haende direkt kommandiert).

KATEGORIEN ("Ordner"): Jede Pose haengt in genau einer Kategorie -- rein
organisatorisch, alles bleibt in DERSELBEN Datei (nur getrennt gefuehrt).
Pose-Namen sind global eindeutig (der Name ist der Schluessel, mit dem
`/g1pilot/pose_store/goto` eine Pose anfaehrt) -- die Kategorie ist Metadatum,
kein Namensraum. Leere Kategorien werden mitgespeichert, damit ein im
Speichern-Dialog angelegter "Ordner" auch ohne Pose darin erhalten bleibt.

Dateiformat (Version 2):
    {"version": 2,
     "categories": ["Allgemein", "Greifen"],
     "poses": {"<name>": {"category": ..., "left_arm": [...], "saved_at": ...}}}
Version 1 (flaches {name: eintrag}) wird transparent gelesen und beim naechsten
Schreiben in Version 2 ueberfuehrt.

Wird von ZWEI Prozessen genutzt (arm_controller.py speichert/laedt, die
Streamdeck-GUI in teleoperation/ui_interface.py listet zum Anzeigen und legt
Kategorien an) -- daher atomares Schreiben (write-tmp-then-rename), damit ein
gleichzeitiges Lesen nie eine halb geschriebene Datei sieht.
"""
import json
import os
import time
from pathlib import Path

# Bind-Mount-Pfad des Repos IM Container (docker-compose: .:/ros2_ws/src/g1pilot).
# Alles hier drunter liegt auf dem HOST und ueberlebt den Container.
CONTAINER_REPO_MOUNT = "/ros2_ws/src/g1pilot"
# Unterverzeichnis fuer Laufzeit-/Nutzerdaten im Repo (nicht versioniert).
REPO_DATA_SUBDIR = "data"
STORE_FILENAME = "arm_poses.json"
# Ablage der frueheren Versionen (fluechtig im Container) -- Quelle der Migration.
LEGACY_PATH = os.path.expanduser(os.path.join("~", ".g1pilot", STORE_FILENAME))
# Voller Pfad-Override fuer Sonderfaelle (Tests, mehrere Roboter, ...).
PATH_ENV_VAR = "G1_POSE_STORE"

# Speicherbare Komponenten (Reihenfolge = Anzeige-/Iterationsreihenfolge).
COMPONENTS = ("left_arm", "right_arm", "left_hand", "right_hand")

# Kategorie fuer Posen ohne explizite Angabe (und fuer Legacy-Eintraege).
DEFAULT_CATEGORY = "Allgemein"

SCHEMA_VERSION = 2


def default_store_path() -> Path:
    """Ablageort der Pose-Datei: `G1_POSE_STORE` > Repo-Mount (persistent) >
    `~/.g1pilot/` (Host-Betrieb ohne Mount)."""
    env = (os.environ.get(PATH_ENV_VAR) or "").strip()
    if env:
        return Path(os.path.expanduser(env))
    mount = Path(CONTAINER_REPO_MOUNT)
    if mount.is_dir():
        return mount / REPO_DATA_SUBDIR / STORE_FILENAME
    return Path(LEGACY_PATH)


def _clean_category(category) -> str:
    name = (category or "").strip()
    return name or DEFAULT_CATEGORY


class PoseStore:
    def __init__(self, path=None):
        self.path = Path(path) if path else default_store_path()
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
        except OSError:
            # Repo read-only gemountet o.ae.: lieber fluechtig unter $HOME
            # weiterlaufen als den arm_controller beim Start hochgehen lassen.
            if path is None and self.path != Path(LEGACY_PATH):
                self.path = Path(LEGACY_PATH)
                self.path.parent.mkdir(parents=True, exist_ok=True)
            else:
                raise
        self._migrate_legacy_file()

    # ── Datei-Ebene ──────────────────────────────────────────────────────
    def _migrate_legacy_file(self) -> None:
        """Einmalige Uebernahme der alten (fluechtigen) Ablage unter $HOME in
        die persistente. Nur wenn am Ziel noch nichts liegt -- die neue Datei
        ist immer die Wahrheit."""
        legacy = Path(LEGACY_PATH)
        if self.path == legacy or self.path.exists() or not legacy.is_file():
            return
        try:
            self.path.write_text(legacy.read_text(encoding="utf-8"), encoding="utf-8")
        except OSError:
            pass   # Migration ist Komfort, kein Muss -- niemals den Start blockieren

    def _read_raw(self) -> dict:
        if not self.path.is_file():
            return {}
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        return data if isinstance(data, dict) else {}

    def _load(self) -> dict:
        """-> normalisiertes Version-2-Dokument (auch fuer Version-1-Dateien)."""
        raw = self._read_raw()
        if not raw:
            return {"version": SCHEMA_VERSION, "categories": [], "poses": {}}
        if isinstance(raw.get("poses"), dict):
            cats = [c for c in raw.get("categories", []) if isinstance(c, str)]
            poses = {k: v for k, v in raw["poses"].items() if isinstance(v, dict)}
            return {"version": SCHEMA_VERSION, "categories": cats, "poses": poses}
        # Version 1: flaches {name: eintrag} ohne Kategorien.
        poses = {k: v for k, v in raw.items() if isinstance(v, dict)}
        return {"version": SCHEMA_VERSION, "categories": [], "poses": poses}

    def _save(self, doc: dict) -> None:
        out = {
            "version": SCHEMA_VERSION,
            "categories": sorted(set(doc.get("categories", []))),
            "poses": doc.get("poses", {}),
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
        os.replace(tmp, self.path)   # atomar auf POSIX -- kein halb geschriebener Zustand sichtbar

    # ── Lesen ────────────────────────────────────────────────────────────
    def list_names(self) -> list:
        return sorted(self._load()["poses"].keys())

    def list_categories(self) -> list:
        """Alle bekannten Kategorien: explizit angelegte + von Posen benutzte.
        DEFAULT_CATEGORY steht immer an erster Stelle, der Rest alphabetisch."""
        doc = self._load()
        cats = set(doc["categories"])
        for entry in doc["poses"].values():
            cats.add(_clean_category(entry.get("category")))
        cats.discard(DEFAULT_CATEGORY)
        return [DEFAULT_CATEGORY] + sorted(cats)

    def list_grouped(self) -> dict:
        """-> {kategorie: [posennamen sortiert]} fuer die Anzeige beim Laden.
        Enthaelt AUCH leere Kategorien (damit angelegte "Ordner" sichtbar
        bleiben). Reihenfolge wie list_categories()."""
        doc = self._load()
        grouped = {c: [] for c in self.list_categories()}
        for name, entry in doc["poses"].items():
            grouped.setdefault(_clean_category(entry.get("category")), []).append(name)
        return {c: sorted(names) for c, names in grouped.items()}

    def category_of(self, name: str):
        """-> Kategorie der Pose `name`, oder None wenn es die Pose nicht gibt."""
        entry = self._load()["poses"].get(name)
        if entry is None:
            return None
        return _clean_category(entry.get("category"))

    def get(self, name: str):
        """-> dict mit den VORHANDENEN Komponenten-Schluesseln (Teilmenge von
        COMPONENTS) oder None, wenn der Name nicht existiert. Legacy-Eintraege
        (alte Zwei-Arm-Form mit 'left'/'right') werden transparent auf
        'left_arm'/'right_arm' abgebildet."""
        entry = self._load()["poses"].get(name)
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

    # ── Schreiben ────────────────────────────────────────────────────────
    def save(self, name: str, *, category=None, left_arm=None, right_arm=None,
             left_hand=None, right_hand=None) -> None:
        """Speichert genau die uebergebenen (nicht-None) Komponenten unter
        `name` in der Kategorie `category` (Default: DEFAULT_CATEGORY).
        Mindestens eine Komponente muss gesetzt sein."""
        if not (name or "").strip():
            raise ValueError("Pose-Name darf nicht leer sein.")
        name = name.strip()
        entry = {"category": _clean_category(category)}
        if left_arm is not None:
            entry["left_arm"] = [float(x) for x in left_arm]
        if right_arm is not None:
            entry["right_arm"] = [float(x) for x in right_arm]
        if left_hand is not None:
            entry["left_hand"] = [float(x) for x in left_hand]
        if right_hand is not None:
            entry["right_hand"] = [float(x) for x in right_hand]
        if len(entry) == 1:
            raise ValueError("Nichts zum Speichern ausgewaehlt.")
        entry["saved_at"] = time.time()
        doc = self._load()
        doc["poses"][name] = entry
        doc["categories"] = list(doc["categories"]) + [entry["category"]]
        self._save(doc)

    def add_category(self, category: str) -> str:
        """Legt eine (zunaechst leere) Kategorie an -- fuer den Button
        "Kategorie anlegen" im Speichern-Dialog, damit der "Ordner" auch dann
        erhalten bleibt, wenn der Dialog abgebrochen wird. -> normierter Name."""
        cat = (category or "").strip()
        if not cat:
            raise ValueError("Kategorie-Name darf nicht leer sein.")
        doc = self._load()
        if cat not in doc["categories"]:
            doc["categories"] = list(doc["categories"]) + [cat]
            self._save(doc)
        return cat

    def delete(self, name: str) -> bool:
        doc = self._load()
        if name in doc["poses"]:
            del doc["poses"][name]
            self._save(doc)
            return True
        return False
