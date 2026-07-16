# -*- coding: utf-8 -*-
"""
grasp_box.py — Live-Toggle der greifbaren Test-Kugel(n).

Die Greif-Box wird beim Laden IMMER ins Modell eingefuegt (siehe unitree_mujoco.py),
aber standardmaessig AUS: keine Kollision (contype/conaffinity = 0) und unsichtbar
(rgba-Alpha = 0), nur ~6 g inert an der Palme. Zur Laufzeit kann man in MuJoCo keinen
Koerper hinzufuegen — aber Kollision und Sichtbarkeit eines vorhandenen Geoms sehr wohl
umschalten. Genau das macht dieser Listener.

Warum UDP statt ROS-Topic? Wie bei push_listener.py: der MuJoCo-Container hat KEIN ROS.
Der Streamdeck-Button (ROS, im g1pilot-Container) wird von loco_sim auf ein UDP-Datagramm
an 127.0.0.1:<port> abgebildet (beide Container: network_mode host -> Loopback).

Protokoll: ein UDP-Datagramm schaltet die Box. Payload "on"/"1"/"an" -> AN,
"off"/"0"/"aus" -> AUS, alles andere (oder leer) -> umschalten (Toggle).
"""
import socket
import threading


class GraspBox:
    def __init__(self, mj_model, config):
        self.model = mj_model
        self.enabled = bool(getattr(config, "GRASP_BOX", False))
        self.port = int(getattr(config, "GRASP_UDP_PORT", 47901))

        # Geom-IDs der Box(en) aufloesen (in unitree_mujoco.py benannt).
        self.gids = []
        for nm in ("left_grasp_box", "right_grasp_box"):
            try:
                self.gids.append(mj_model.geom(nm).id)
            except Exception:
                pass

        self._state = bool(getattr(config, "GRASP_TEST", False))   # aktueller Zustand
        self._desired = self._state
        self._dirty = False
        self._lock = threading.Lock()

        if not self.enabled or not self.gids:
            if self.enabled and not self.gids:
                print("[graspbox] keine Box-Geome im Modell gefunden -> Toggle inaktiv.")
            return

        # Start-Zustand erzwingen: die Box wird IMMER greifbar (contype=2) kompiliert
        # (sonst faellt sie aus dem Kollisions-Baum). Ist der Start AUS, hier einmal
        # auf inert schalten. rgba passt schon aus der Injektion, wird aber mitgesetzt.
        self._set(self._state)

        self._sock = None
        threading.Thread(target=self._listen, daemon=True).start()
        print(f"[graspbox] UDP-Toggle aktiv auf 127.0.0.1:{self.port} "
              f"(Start {'AN' if self._state else 'AUS'}; Streamdeck-Button 'GRASP BOX').")

    def _set(self, want: bool) -> None:
        """Kollision (Bit 2 <-> 0) und Sichtbarkeit (Alpha) der Box(en) setzen."""
        ct = 2 if want else 0
        alpha = 1.0 if want else 0.0
        for gid in self.gids:
            self.model.geom_contype[gid] = ct
            self.model.geom_conaffinity[gid] = ct
            self.model.geom_rgba[gid][3] = alpha
        self._state = want

    # ── UDP-Thread: wartet auf Schalt-Befehle ────────────────────────────────
    def _listen(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("127.0.0.1", self.port))
        except Exception as e:
            print(f"[graspbox] WARN: konnte UDP-Port {self.port} nicht binden: {e}")
            return
        while True:
            try:
                data, _ = self._sock.recvfrom(64)
            except Exception:
                break
            txt = data.decode("utf-8", "ignore").strip().lower()
            if txt in ("on", "1", "true", "yes", "an"):
                want = True
            elif txt in ("off", "0", "false", "no", "aus"):
                want = False
            else:
                want = not self._state   # ohne klaren Payload: umschalten
            with self._lock:
                self._desired = want
                self._dirty = True

    # ── Pro Sim-Schritt aufrufen (unter locker) ──────────────────────────────
    def apply(self):
        """Wendet einen ausstehenden Toggle auf mj_model an (Kollision + Sichtbarkeit).
        Guenstig: tut nur etwas, wenn ein Befehl anliegt."""
        if not self.gids:
            return
        with self._lock:
            if not self._dirty:
                return
            want = self._desired
            self._dirty = False
        self._set(want)
        print(f"[graspbox] Greif-Box {'AN (greifbar+sichtbar)' if want else 'AUS (inert)'}.",
              flush=True)
