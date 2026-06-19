# -*- coding: utf-8 -*-
"""
push_listener.py — Stoer-Impuls ("Schubs") fuer den Balancer-Test.

Hoert auf einem lokalen UDP-Port (127.0.0.1) auf einen Trigger und bringt
dann fuer eine kurze Dauer eine externe Kraft in ZUFAELLIGER horizontaler
Richtung auf einen Koerper (Default: torso_link) auf — so laesst sich pruefen,
wie gut der Balancer eine Stoerung ausgleicht.

Warum UDP statt ROS-Topic? Der MuJoCo-Container hat KEIN ROS (nur mujoco +
Unitree-DDS). Der Streamdeck-Button (ROS, im g1pilot-Container) wird von
loco_sim auf ein UDP-Datagramm an 127.0.0.1:<port> abgebildet. Beide Container
laufen im network_mode: host -> Loopback verbindet sie ohne DDS-IDL/ROS hier.

Protokoll: Ein UDP-Datagramm = ein Stoss. Leerer/nicht-numerischer Payload
-> Kraft aus config (PUSH_FORCE_N). Enthaelt der Payload eine Zahl, wird sie
als Kraft [N] verwendet (CLI-Tuning ohne Neustart). Die RICHTUNG ist immer
zufaellig (gleichverteilt in der Horizontalen).
"""
import math
import random
import socket
import threading
import time

import numpy as np


class PushListener:
    def __init__(self, mj_model, config):
        self.enabled = bool(getattr(config, "PUSH_ENABLE", True))
        self.port = int(getattr(config, "PUSH_UDP_PORT", 47900))
        self.force_n = float(getattr(config, "PUSH_FORCE_N", 150.0))
        self.duration = float(getattr(config, "PUSH_DURATION_S", 0.12))
        body_name = str(getattr(config, "PUSH_BODY", "torso_link"))

        self._lock = threading.Lock()
        self._force = np.zeros(3)     # aktuell aufgebrachte Kraft (Welt-KS)
        self._until = 0.0             # perf_counter-Zeit, bis zu der gedrueckt wird
        self._active = False          # ob xfrc gerade von uns gesetzt ist

        # Koerper-ID aufloesen (mit Fallbacks, falls das Modell anders heisst).
        self.body_id = None
        for name in (body_name, "torso_link", "pelvis", "base_link"):
            try:
                self.body_id = mj_model.body(name).id
                self.body_name = name
                break
            except Exception:
                continue

        if not self.enabled:
            print("[push] deaktiviert (PUSH_ENABLE=0).")
            return
        if self.body_id is None:
            print("[push] WARN: kein passender Koerper gefunden -> Stoss deaktiviert.")
            self.enabled = False
            return

        self._sock = None
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()
        print(f"[push] UDP-Trigger aktiv auf 127.0.0.1:{self.port} -> Stoss auf "
              f"'{self.body_name}', {self.force_n:.0f} N fuer {self.duration*1000:.0f} ms, "
              f"Richtung zufaellig.")

    # ── UDP-Thread: wartet auf Trigger ───────────────────────────────────────
    def _listen(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("127.0.0.1", self.port))
        except Exception as e:
            print(f"[push] WARN: konnte UDP-Port {self.port} nicht binden: {e}")
            self.enabled = False
            return
        while True:
            try:
                data, _ = self._sock.recvfrom(64)
            except Exception:
                break
            # Optionaler Payload = Kraft [N]; sonst config-Default.
            mag = self.force_n
            try:
                txt = data.decode("utf-8", "ignore").strip()
                if txt:
                    mag = float(txt)
            except (ValueError, UnicodeDecodeError):
                pass
            self._trigger(mag)

    def _trigger(self, magnitude):
        angle = random.uniform(0.0, 2.0 * math.pi)
        fx = magnitude * math.cos(angle)
        fy = magnitude * math.sin(angle)
        with self._lock:
            self._force = np.array([fx, fy, 0.0])
            self._until = time.perf_counter() + self.duration
        print(f"[push] SCHUBS! {magnitude:.0f} N @ {math.degrees(angle):6.1f}deg "
              f"(fx={fx:+.0f}, fy={fy:+.0f}) fuer {self.duration*1000:.0f} ms.")

    # ── Pro Sim-Schritt aufrufen: setzt/loescht xfrc_applied ─────────────────
    def apply(self, mj_data):
        if not self.enabled:
            return
        with self._lock:
            now = time.perf_counter()
            if now < self._until:
                mj_data.xfrc_applied[self.body_id, 0:3] = self._force
                self._active = True
            elif self._active:
                # Stoss vorbei -> Kraft genau einmal zuruecknehmen.
                mj_data.xfrc_applied[self.body_id, 0:3] = 0.0
                self._active = False
