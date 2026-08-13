#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_api.py — HTTP-JSON-Bruecke fuer die Live-Pose-Schnittstelle.
Vollstaendige Doku inkl. Beispielen: g1pilot/docs/21_arm_api_technik.md

WOZU: Ein fremdes Projekt berechnet eine Armposition und will sie ausfuehren
lassen. Ueber ROS geht das schon (`/g1pilot/arm_command`), aber der Aufrufer
braeuchte dafuer ROS 2 Humble, dieselbe ROS_DOMAIN_ID und die CycloneDDS-Config
des Containers. Diese Bruecke macht daraus einen gewoehnlichen HTTP-POST mit
JSON -- aus jeder Sprache, ohne ROS, mit ECHTER Antwort (angenommen/abgelehnt +
Grund, optional bis "erreicht" wartend).

ARCHITEKTUR (bewusst ein EIGENER Node):
    Client --HTTP--> arm_api --/g1pilot/arm_command--> arm_controller (250 Hz)
                            <--/g1pilot/arm_command/status--
Der arm_controller regelt mit 250 Hz; dort hat kein Webserver etwas zu suchen.
Genauso ist die scene_bridge getrennt. Die Bruecke selbst haelt KEINE Logik:
sie validiert per manipulation/arm_command.py (dieselbe Funktion, die auch der
Controller nutzt -> kein Auseinanderlaufen), publiziert und sammelt Status.

SICHERHEIT: Default-Bind ist 127.0.0.1 -- die Schnittstelle fahrt einen echten
Roboterarm, sie gehoert nicht ungeschuetzt ins Netz. Fuer einen anderen Rechner
`bind_host` bewusst oeffnen UND `auth_token` setzen (Header `X-Auth-Token`).
Alle Sicherheitsgates (E-Stop, ENABLE MANIPULATION, Kollision, Limits) sitzen
im arm_controller, nicht hier -- diese Bruecke kann sie nicht umgehen.
"""
from __future__ import annotations

import http.server
import json
import threading
import time
import urllib.parse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from g1pilot.manipulation import arm_command as ac
from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    LEFT_JOINT_INDICES_LIST,
    RIGHT_JOINT_INDICES_LIST,
)

# Wie lange auf die erste Reaktion des Controllers gewartet wird (accepted/
# rejected). Der Controller antwortet innerhalb eines Ticks; 2 s sind reichlich
# und fangen den Fall "arm_controller laeuft nicht" sauber ab.
ACCEPT_TIMEOUT_S = 2.0
# Obergrenze fuer ?wait= (Planung + Abfahren kann dauern, aber nicht endlos).
MAX_WAIT_S = 300.0


class _Vorgang:
    """Ein eingespieltes Kommando und sein Status-Verlauf."""

    def __init__(self, req_id: str):
        self.id = req_id
        self.states = []                  # Liste der Status-dicts, aeltestes zuerst
        self.terminal = threading.Event()  # gesetzt, sobald ein Endzustand kam
        self.answered = threading.Event()  # gesetzt bei der ERSTEN Meldung
        self.created = time.time()

    @property
    def last(self):
        return self.states[-1] if self.states else None

    def add(self, status: dict) -> None:
        self.states.append(status)
        self.answered.set()
        if status.get("state") in ac.ALL_TERMINAL_STATES:
            self.terminal.set()


class ArmApiNode(Node):
    """ROS-Seite der Bruecke: publiziert Kommandos, sammelt Status je Vorgang."""

    def __init__(self):
        super().__init__("arm_api")
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("port", 8770)
        # Leer = keine Pruefung (nur zusammen mit bind_host=127.0.0.1 vertretbar).
        self.declare_parameter("auth_token", "")
        # Wie viele abgeschlossene Vorgaenge im Speicher bleiben (fuer GET /status).
        self.declare_parameter("history", 200)

        self.pub_cmd = self.create_publisher(String, "/g1pilot/arm_command", 10)
        self.pub_cancel = self.create_publisher(Bool, "/g1pilot/arm_command/cancel", 10)
        self.create_subscription(String, "/g1pilot/arm_command/status",
                                 self._on_status, 50)
        # Speichern laeuft ueber das Topic, das auch die Streamdeck-GUI nutzt --
        # eine Implementierung, zwei Aufrufer (siehe arm_controller._on_pose_save).
        self.pub_save = self.create_publisher(String, "/g1pilot/pose_store/save", 10)
        self.create_subscription(String, "/g1pilot/pose_store/save/status",
                                 self._on_status, 50)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self._lock = threading.Lock()
        self._vorgaenge = {}      # id -> _Vorgang
        self._order = []          # Einfuegereihenfolge, fuer das Aufraeumen
        self._last_status = None  # letzte Meldung, auch ohne passenden Vorgang
        self._joints = None       # {"left": [7], "right": [7], "stamp": float}

        self.token = str(self.get_parameter("auth_token").value or "")
        self.history = int(self.get_parameter("history").value)

    # ── ROS-Eingang ──────────────────────────────────────────────────────
    def _on_status(self, msg: String):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(status, dict):
            return
        status["received_at"] = time.time()
        with self._lock:
            self._last_status = status
            vg = self._vorgaenge.get(status.get("id") or "")
            if vg is not None:
                vg.add(status)

    def _on_joint_states(self, msg: JointState):
        """Ist-Gelenkwinkel der Arme mitschreiben -- der Aufrufer braucht seine
        Startpose (und will nach dem Anfahren nachrechnen koennen)."""
        try:
            index = {name: i for i, name in enumerate(msg.name)}
            out = {}
            for side, idx_list in (("left", LEFT_JOINT_INDICES_LIST),
                                   ("right", RIGHT_JOINT_INDICES_LIST)):
                vals = []
                for jidx in idx_list:
                    pos = index.get(JOINT_NAMES_ROS[jidx])
                    if pos is None or pos >= len(msg.position):
                        vals = None
                        break
                    vals.append(round(float(msg.position[pos]), 6))
                if vals is not None:
                    out[side] = vals
            if out:
                out["stamp"] = time.time()
                with self._lock:
                    self._joints = out
        except Exception:   # noqa: BLE001 -- Telemetrie darf den Node nie stoeren
            pass

    # ── Von der HTTP-Seite benutzt ───────────────────────────────────────
    def submit(self, wire: dict, cmd: dict, *, save: bool = False) -> _Vorgang:
        """Kommando publizieren und den Vorgang registrieren.

        Auf das Topic geht das WIRE-Format (`wire`, so wie der Client es
        geschickt hat, nur mit gesetzter id/type) -- NICHT das normalisierte
        `cmd`: der arm_controller parst mit derselben parse_command()-Funktion
        und erwartet daher dieselbe Struktur. `cmd` dient hier nur dazu, vorab
        abzulehnen und die id/sides zu kennen.

        Registrierung VOR dem Publish, sonst kann die Status-Antwort das
        Rennen gewinnen und ins Leere laufen."""
        vg = _Vorgang(cmd["id"])
        with self._lock:
            self._vorgaenge[vg.id] = vg
            self._order.append(vg.id)
            while len(self._order) > max(10, self.history):
                old = self._order.pop(0)
                self._vorgaenge.pop(old, None)
        if save:
            self.pub_save.publish(String(data=json.dumps(wire)))
            self.get_logger().info(
                f"Pose '{cmd['name']}' -> Kategorie "
                f"'{cmd['category'] or '(Default)'}' per HTTP zum Speichern "
                f"angefordert ({', '.join(cmd['components'])}).")
        else:
            self.pub_cmd.publish(String(data=json.dumps(wire)))
            self.get_logger().info(
                f"arm_command '{vg.id}' ({cmd['type']}, {'+'.join(cmd['sides'])}) "
                f"per HTTP eingespielt.")
        return vg

    def cancel(self) -> None:
        self.pub_cancel.publish(Bool(data=True))
        self.get_logger().info("Abbruch per HTTP angefordert.")

    def vorgang(self, req_id: str):
        with self._lock:
            return self._vorgaenge.get(req_id)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "last_status": self._last_status,
                "joints": self._joints,
                "open": [i for i, v in self._vorgaenge.items() if not v.terminal.is_set()],
            }


class _Handler(http.server.BaseHTTPRequestHandler):
    """HTTP-Endpunkte. `node` wird von main() als Klassenattribut gesetzt."""

    node: ArmApiNode = None
    protocol_version = "HTTP/1.1"
    server_version = "g1pilot-arm-api/1.0"

    # ── Infrastruktur ────────────────────────────────────────────────────
    def log_message(self, fmt, *args):
        # Zugriffe in das ROS-Log statt auf stderr (sonst zwei Log-Stroeme).
        if self.node is not None:
            self.node.get_logger().debug("HTTP " + (fmt % args))

    def _send(self, code: int, payload: dict):
        body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self) -> bool:
        token = self.node.token if self.node else ""
        if not token:
            return True
        return self.headers.get("X-Auth-Token", "") == token

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            raise ac.CommandError("Content-Length fehlt oder ist ungueltig.")
        if length <= 0:
            raise ac.CommandError("Leerer Request-Body -- erwarte JSON.")
        if length > 1 << 20:
            raise ac.CommandError("Request-Body zu gross.")
        raw = self.rfile.read(length)
        try:
            obj = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError) as e:
            raise ac.CommandError(f"Kein gueltiges JSON: {e}")
        if not isinstance(obj, dict):
            raise ac.CommandError("Erwarte ein JSON-Objekt.")
        return obj

    @staticmethod
    def _wait_seconds(obj: dict) -> float:
        """'wait': Sekunden bis zum ENDZUSTAND (0/fehlt = nur bis angenommen)."""
        val = obj.get("wait", 0)
        if isinstance(val, bool) or not isinstance(val, (int, float)):
            raise ac.CommandError("'wait': erwarte Sekunden als Zahl.")
        return max(0.0, min(float(val), MAX_WAIT_S))

    # ── Routen ───────────────────────────────────────────────────────────
    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path.rstrip("/") or "/"
        if path == "/":
            self._send(200, self._describe())
        elif path == "/arm/health":
            self._send(200, {"ok": True, "node": "arm_api"})
        elif path == "/arm/state":
            if not self._authorized():
                self._send(401, {"error": "Token fehlt oder falsch."})
                return
            self._send(200, self.node.snapshot())
        elif path.startswith("/arm/status"):
            if not self._authorized():
                self._send(401, {"error": "Token fehlt oder falsch."})
                return
            rest = path[len("/arm/status"):].strip("/")
            if not rest:
                self._send(200, {"last_status": self.node.snapshot()["last_status"]})
                return
            vg = self.node.vorgang(rest)
            if vg is None:
                self._send(404, {"error": f"Vorgang '{rest}' unbekannt.", "id": rest})
                return
            self._send(200, {"id": vg.id, "state": (vg.last or {}).get("state"),
                             "history": vg.states})
        else:
            self._send(404, {"error": f"Unbekannter Pfad '{path}'.",
                             "hint": "GET / listet die Endpunkte."})

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path.rstrip("/") or "/"
        if not self._authorized():
            self._send(401, {"error": "Token fehlt oder falsch."})
            return
        if path == "/arm/cancel":
            self.node.cancel()
            self._send(200, {"cancelled": True})
            return
        if path == "/arm/save":
            self._do_save()
            return
        if path not in ("/arm/joints", "/arm/pose"):
            self._send(404, {"error": f"Unbekannter Pfad '{path}'.",
                             "hint": "POST /arm/joints, /arm/pose, /arm/save "
                                     "oder /arm/cancel."})
            return

        try:
            obj = self._read_json()
            wait_s = self._wait_seconds(obj)
            # Der Pfad bestimmt die Variante -- ein 'type' im Body darf nicht
            # widersprechen (sonst faehrt jemand versehentlich Gelenkwinkel als
            # kartesisches Ziel oder umgekehrt).
            want = ac.TYPE_JOINTS if path == "/arm/joints" else ac.TYPE_POSE
            if obj.get("type") not in (None, want):
                raise ac.CommandError(
                    f"'type' ({obj['type']}) passt nicht zu {path} (erwarte '{want}').")
            obj["type"] = want
            cmd = ac.parse_command(obj)
            # Die (ggf. generierte) id muss mit ins Wire-Format, sonst kann der
            # Status des Controllers dem Vorgang nicht zugeordnet werden.
            obj["id"] = cmd["id"]
            obj.pop("wait", None)          # rein HTTP-seitig, gehoert nicht aufs Topic
        except ac.CommandError as e:
            self._send(400, {"error": str(e)})
            return

        vg = self.node.submit(obj, cmd)

        # Immer mindestens bis zur ersten Reaktion warten -- ohne die weiss der
        # Aufrufer nicht, ob sein Ziel ueberhaupt angenommen wurde.
        if not vg.answered.wait(ACCEPT_TIMEOUT_S):
            self._send(504, {
                "id": vg.id, "state": "unknown",
                "error": "Keine Antwort vom arm_controller -- laeuft der Node?"})
            return
        if wait_s > 0:
            vg.terminal.wait(wait_s)

        last = vg.last or {}
        state = last.get("state")
        if state == ac.ST_REJECTED:
            code = 409          # Konflikt: syntaktisch ok, aber jetzt nicht ausfuehrbar
        elif state == ac.ST_FAILED:
            code = 422          # nicht erreichbar (IK/Planung)
        elif state in (ac.ST_REACHED, ac.ST_CANCELLED):
            code = 200
        else:
            code = 202          # laeuft noch (accepted/executing)
        self._send(code, {"id": vg.id, "state": state,
                          "reason": last.get("reason", ""),
                          "detail": last.get("detail"),
                          "sides": cmd["sides"],
                          "history": vg.states})

    def _do_save(self):
        """POST /arm/save — AKTUELLE Stellung in die Pose-Datei schreiben.

        Bewegt nichts. Geht auf dasselbe Topic wie der Streamdeck-Speichern-
        Dialog, also dieselbe Implementierung und dieselbe Datei; der Rueckkanal
        sagt, WAS tatsaechlich gespeichert wurde (Haende brauchen eine laufende
        inspire-Bridge). Gespeichert wird immer sofort -- kein `wait` noetig, die
        Antwort kommt innerhalb eines Ticks."""
        try:
            obj = self._read_json()
            cmd = ac.parse_save_command(obj)
            if not cmd["id"]:
                cmd["id"] = ac.new_request_id()
            obj["id"] = cmd["id"]     # ohne id kann der Status nicht zugeordnet werden
        except ac.CommandError as e:
            self._send(400, {"error": str(e)})
            return

        vg = self.node.submit(obj, cmd, save=True)
        if not vg.answered.wait(ACCEPT_TIMEOUT_S):
            self._send(504, {
                "id": vg.id, "state": "unknown",
                "error": "Keine Antwort vom arm_controller -- laeuft der Node?"})
            return
        last = vg.last or {}
        state = last.get("state")
        code = {ac.ST_SAVED: 200, ac.ST_REJECTED: 400, ac.ST_FAILED: 422}.get(state, 202)
        self._send(code, {
            "id": vg.id, "state": state, "name": last.get("name", cmd["name"]),
            "category": last.get("category", ""),
            "requested": cmd["components"],
            "components": last.get("components", []),
            "skipped": last.get("skipped", []),
            "reason": last.get("reason", ""),
        })

    def _describe(self) -> dict:
        return {
            "service": "g1pilot arm API",
            "doc": "g1pilot/docs/21_arm_api_technik.md",
            "endpoints": {
                "POST /arm/joints": "Gelenkwinkel (7 je Arm, rad) direkt anfahren",
                "POST /arm/pose": "Kartesische Hand-Pose anfahren (IK + Planung)",
                "POST /arm/cancel": "laufende Bewegung abbrechen",
                "POST /arm/save": "aktuelle Stellung in die Pose-Datei speichern "
                                  "(name, category, components)",
                "GET /arm/status/<id>": "Status eines Vorgangs",
                "GET /arm/state": "Ist-Gelenkwinkel + letzter Status",
                "GET /arm/health": "lebt der Node?",
            },
            "beispiel": {
                "POST /arm/pose": {
                    "right": {"position": [0.35, -0.20, 0.10],
                              "rpy_deg": [0, 0, 0], "frame": "pelvis"},
                    "wait": 60,
                },
                "POST /arm/save": {
                    "name": "Regal oben", "category": "Greifen",
                    "components": ["arms", "hands"],
                },
            },
        }


def main():
    rclpy.init()
    node = ArmApiNode()
    _Handler.node = node

    host = str(node.get_parameter("bind_host").value)
    port = int(node.get_parameter("port").value)
    httpd = http.server.ThreadingHTTPServer((host, port), _Handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    node.get_logger().info(
        f"Arm-API auf http://{host}:{port} (POST /arm/joints | /arm/pose, "
        f"Doku: g1pilot/docs/21_arm_api_technik.md)"
        + ("" if node.token else "  [ohne Token -- nur fuer lokalen Zugriff]"))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
