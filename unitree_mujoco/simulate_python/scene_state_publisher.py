# -*- coding: utf-8 -*-
"""
scene_state_publisher.py — sendet die Umgebungs-Objekte (Hindernisse + greif-
bare Objekte) periodisch per UDP an den ROS-Container, damit sie dort in RViz
angezeigt und von Nav/IK als Kollisionsgeometrie genutzt werden koennen (siehe
g1pilot/g1pilot/navigation/scene_bridge.py, der Empfaenger).

Warum UDP statt ROS-Topic? Wie bei push_listener.py/grasp_box.py: der MuJoCo-
Container hat KEIN ROS. Beide Container laufen mit network_mode: host ->
Loopback verbindet sie ohne DDS-IDL/ROS hier. Anders als bei den bestehenden
Listenern ist die Richtung hier umgekehrt (MuJoCo -> ROS statt ROS -> MuJoCo)
und die Payload reicher (volle Objektliste statt eines Toggle-Bits) -> JSON
statt eines einzelnen Bytes/Zahl.

Protokoll: ein UDP-Datagramm = eine komplette Szenen-Momentaufnahme (JSON):
    {"obstacles": [...], "grasp": [...]}
Jedes Objekt: {name, class, type, pos[3], quat[4 wxyz], size/mesh_basename/
aabb_half, rgba[4]}. Hindernisse haben eine FESTE Pose (aus der Szenen-XML,
einmalig geparst); greifbare Objekte werden JEDEN Publish-Tick mit ihrer
LIVE-Pose aus mj_data aktualisiert (sie sind freie Koerper -- siehe
build_env_scene.py -- und bewegen sich beim Greifen/Anfassen).
"""
import json
import os
import socket
import time

import scene_objects


class SceneStatePublisher:
    def __init__(self, mj_model, config):
        self.enabled = bool(getattr(config, "SCENE_ENABLE", True))
        self.port = int(getattr(config, "SCENE_UDP_PORT", 47902))
        self.host = str(getattr(config, "SCENE_UDP_HOST", "127.0.0.1"))
        self.hz = float(getattr(config, "SCENE_PUBLISH_HZ", 10.0))
        self._period = 1.0 / self.hz if self.hz > 0 else 0.1
        self._next_t = 0.0
        self._sock = None
        self._warned_size = False

        self.obstacles = []
        self.grasp = []   # je Eintrag: dict + "body_id"

        if not self.enabled:
            print("[scene] deaktiviert (SCENE_ENABLE=0).")
            return

        scene_path = os.path.abspath(config.ROBOT_SCENE)
        try:
            specs = scene_objects.parse_scene_objects(scene_path)
        except Exception as e:
            print(f"[scene] WARN: Szene konnte nicht geparst werden ({e}) -> "
                  f"keine Szenen-Objekte in RViz/Nav/IK.")
            specs = []

        for spec in specs:
            if spec["class"] == "grasp":
                try:
                    body_id = mj_model.body(spec["name"]).id
                except Exception:
                    print(f"[scene] WARN: Greif-Objekt '{spec['name']}' nicht im "
                          f"kompilierten Modell gefunden -> uebersprungen "
                          f"(braucht <freejoint/>, siehe build_env_scene.py).")
                    continue
                spec["body_id"] = body_id
                self.grasp.append(spec)
            else:
                self.obstacles.append(spec)

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as e:
            print(f"[scene] WARN: UDP-Socket konnte nicht angelegt werden: {e}")
            self.enabled = False
            return

        print(f"[scene] Szenen-Publisher aktiv: {len(self.obstacles)} Hindernis(se), "
              f"{len(self.grasp)} Greif-Objekt(e) -> 127.0.0.1:{self.port} @ {self.hz:.0f} Hz "
              f"(scene_bridge, ROS-Seite).")

    def _obstacle_payload(self):
        return [
            {
                "name": o["name"], "class": "obstacle", "type": o["type"],
                "pos": o["pos"], "quat": o["quat"], "rgba": o["rgba"],
                "size": o["size"], "mesh": o["mesh_basename"],
                "aabb_half": o["aabb_half"],
            }
            for o in self.obstacles
        ]

    def _grasp_payload(self, mj_data):
        out = []
        for g in self.grasp:
            bid = g["body_id"]
            pos = mj_data.xpos[bid]
            quat = mj_data.xquat[bid]   # MuJoCo: (w, x, y, z)
            out.append({
                "name": g["name"], "class": "grasp", "type": g["type"],
                "pos": [float(pos[0]), float(pos[1]), float(pos[2])],
                "quat": [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])],
                "rgba": g["rgba"], "size": g["size"], "mesh": g["mesh_basename"],
                "aabb_half": g["aabb_half"],
            })
        return out

    def maybe_publish(self, mj_data):
        """Pro Sim-Schritt aufrufen (billig: tut ausserhalb des Zeitrasters
        nichts). Sendet nie eine Exception in die Physik-Schleife weiter."""
        if not self.enabled or self._sock is None:
            return
        now = time.perf_counter()
        if now < self._next_t:
            return
        self._next_t = now + self._period
        try:
            payload = json.dumps({
                "obstacles": self._obstacle_payload(),
                "grasp": self._grasp_payload(mj_data),
            }).encode("utf-8")
        except Exception as e:
            print(f"[scene] WARN: Snapshot konnte nicht kodiert werden: {e}")
            return
        if len(payload) > 60000 and not self._warned_size:
            self._warned_size = True
            print(f"[scene] WARN: Szenen-Snapshot ist {len(payload)} Bytes -- "
                  f"koennte auf manchen Systemen als UDP-Datagramm fragmentieren.")
        try:
            self._sock.sendto(payload, (self.host, self.port))
        except OSError:
            pass   # Empfaenger (scene_bridge) laeuft evtl. gerade nicht -- kein Problem.
