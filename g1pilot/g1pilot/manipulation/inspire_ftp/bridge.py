#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bridge.py — Inspire-FTP-Hand-Bridge (Sim UND echte Haende)
==========================================================
Vereint den frueheren ftp_hand_controller (zwei eigenstaendige Python-Skripte,
die per Modbus TCP mit den echten Haenden sprachen) in EINEM ROS2-Node:

  * WebSocket :8766  <-> Controller/hand_controller_viewer.html  (Steuerung)
  * WebSocket :8765  <-> Viewer/inspire_hand_viewer.html         (Kraft/Taktil)
  * ROS2 /joint_states                                            (RViz-Finger)

Die eigentliche "Hardware"-Anbindung steckt im Backend (siehe backends.py):
Stufe 1 = SimJointStateBackend (RViz), Stufe 2 = MujocoContactBackend
(Sim-Physik, Default), Stufe 3 = InspireModbusBackend (ECHTE Haende via
Modbus TCP, backend:=modbus + left_host/right_host). GUIs identisch.

WICHTIG: Wenn dieser Node laeuft, muss robot_state mit publish_hand_joints:=false
gestartet werden, sonst ueberschreiben sich die beiden /joint_states-Quellen fuer
die Finger gegenseitig (so auch im robot_state.py-Kommentar vermerkt).
"""

from __future__ import annotations

import asyncio
import functools
import http.server
import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    import websockets
except ImportError:  # pragma: no cover
    raise SystemExit("Fehlt: pip install websockets")

from . import joint_map, tactile
from .backends import SimJointStateBackend
from .model import HandModel


class InspireFtpBridge(Node):
    def __init__(self):
        super().__init__("inspire_ftp_bridge")

        # ── Parameter ────────────────────────────────────────────────────────
        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("controller_port", 8766)
        self.declare_parameter("viewer_port", 8765)
        # HTTP-Port, unter dem die beiden HTML-GUIs ausgeliefert werden
        # (http://<host>:8767/hand_controller_viewer.html). file://-URLs mit
        # ?autoconnect=1 scheitern je nach System am Opener (xdg-open/WSL
        # behandeln den Query-String als Teil des Dateinamens) -> darum servieren
        # wir die GUIs direkt aus der Bridge. 0 = aus.
        self.declare_parameter("http_port", 8767)
        self.declare_parameter("update_rate_hz", 50.0)
        # backend: "mujoco" = Stufe 2 (echte Finger-Physik + Touch-Kraefte via DDS),
        #          "sim"    = Stufe 1 (open-loop /joint_states, Kraft/Taktil = 0),
        #          "modbus" = Stufe 3 (ECHTE Haende via Modbus TCP, Roboter-LAN).
        self.declare_parameter("backend", "mujoco")
        self.declare_parameter("interface", "lo")        # DDS-Interface fuer 'mujoco'
        # Sim: nur kosmetische GUI-Anzeige. Modbus-Backend: ECHTE Ziel-IPs der
        # Haende im Roboter-LAN (Standard laut reference/: .210 links, .211
        # rechts, Port 6000). Leer lassen = diese Seite nicht ansteuern.
        self.declare_parameter("left_host", "sim://left")
        self.declare_parameter("right_host", "sim://right")
        self.declare_parameter("modbus_port", 6000)
        # Poll-Rate des Modbus-I/O-Threads je Hand + Taktil-Leserate
        # (tactile_every: jede n-te Runde alle 17 Zonen lesen; 0 = aus).
        self.declare_parameter("hand_poll_rate_hz", 20.0)
        self.declare_parameter("tactile_every", 2)

        self.ws_host = self.get_parameter("ws_host").get_parameter_value().string_value
        self.controller_port = int(self.get_parameter("controller_port").value)
        self.viewer_port = int(self.get_parameter("viewer_port").value)
        rate = float(self.get_parameter("update_rate_hz").value)
        backend_name = self.get_parameter("backend").get_parameter_value().string_value
        interface = self.get_parameter("interface").get_parameter_value().string_value
        left_host = self.get_parameter("left_host").get_parameter_value().string_value
        right_host = self.get_parameter("right_host").get_parameter_value().string_value

        # ── /joint_states-Publisher (nur die 24 Finger-Gelenke) ──────────────
        self.joint_pub = self.create_publisher(JointState, "/joint_states", QoSProfile(depth=10))

        # ── Streamdeck-Hand-Buttons (OPEN/CLOSE LEFT/RIGHT HAND) ─────────────
        #  Streamdeck (ui_interface) und loco_client publizieren "open"/"close" als
        #  String auf diese Topics. Diese Bridge bildet sie auf die 6 Inspire-DOF ab.
        self.create_subscription(
            String, "/g1pilot/hand_action/left",
            lambda m: self._on_hand_action("left", m), 10)
        self.create_subscription(
            String, "/g1pilot/hand_action/right",
            lambda m: self._on_hand_action("right", m), 10)

        # ── Mapping-Limits an die echte URDF klemmen, falls auffindbar ───────
        urdf = joint_map.default_urdf_path()
        limits = joint_map.load_limits_from_urdf(urdf) if urdf else {}
        closed_rad = joint_map._closed_rad(limits)
        if urdf:
            self.get_logger().info(f"URDF-Limits geladen: {urdf}")
        else:
            self.get_logger().warn("URDF nicht gefunden -> CLOSED_RAD-Defaults (ungeklemmt).")

        # ── Modelle + Backend ────────────────────────────────────────────────
        self.models = {
            "left":  HandModel("links",  left_host),
            "right": HandModel("rechts", right_host),
        }
        if backend_name == "modbus":
            from .backends import InspireModbusBackend
            self.backend = InspireModbusBackend(
                self._publish_joint_states,
                hosts={"left": left_host, "right": right_host},
                models=self.models,
                port=int(self.get_parameter("modbus_port").value),
                closed_rad=closed_rad,
                poll_rate_hz=float(self.get_parameter("hand_poll_rate_hz").value),
                tactile_every=int(self.get_parameter("tactile_every").value),
            )
            active = ", ".join(f"{s}={w.client.host}:{w.client.port}"
                               for s, w in self.backend.workers.items()) or "KEINE"
            self.get_logger().info(
                f"Backend=modbus (Stufe 3, ECHTE Haende): {active}")
            if not self.backend.workers:
                self.get_logger().error(
                    "modbus-Backend ohne gueltige Hand-IPs! left_host/right_host "
                    "setzen (z.B. 192.168.123.210/.211).")
        elif backend_name == "mujoco":
            from .backends import MujocoContactBackend
            self.backend = MujocoContactBackend(
                self._publish_joint_states, interface=interface, closed_rad=closed_rad)
            self.get_logger().info(
                f"Backend=mujoco (Stufe 2): DDS rt/inspire/cmd|state ueber '{interface}' "
                f"-> echte Finger-Winkel + Touch-Kraefte.")
        else:
            if backend_name != "sim":
                self.get_logger().warn(
                    f"backend='{backend_name}' unbekannt -> nutze 'sim' (Stufe 1).")
            self.backend = SimJointStateBackend(self._publish_joint_states, closed_rad=closed_rad)

        # ── HTTP-Server fuer die HTML-GUIs ───────────────────────────────────
        self._httpd = None
        http_port = int(self.get_parameter("http_port").value)
        if http_port > 0:
            self._start_http_server(http_port)

        # ── Update-Schleife (rclpy-Timer): Backend treiben + /joint_states ───
        self._last = time.monotonic()
        self.create_timer(1.0 / max(rate, 1.0), self._step)

        self.get_logger().info(
            f"Inspire-FTP-Bridge aktiv | Controller ws://{self.ws_host}:{self.controller_port} | "
            f"Viewer ws://{self.ws_host}:{self.viewer_port} | Backend={backend_name}"
        )

    def _web_dir(self):
        """Verzeichnis mit den HTML-GUIs finden: installiertes share/, sonst
        Quellbaum (docker-compose mountet das Repo -> symlink-install)."""
        candidates = []
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(
                get_package_share_directory("g1pilot"), "manipulation", "inspire_ftp", "web"))
        except Exception:  # noqa: BLE001
            pass
        candidates.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "web"))
        for c in candidates:
            if os.path.isfile(os.path.join(c, "hand_controller_viewer.html")):
                return c
        return None

    def _start_http_server(self, port):
        web = self._web_dir()
        if web is None:
            self.get_logger().warn("HTML-GUIs nicht gefunden -> kein HTTP-Server.")
            return
        handler = functools.partial(_QuietHttpHandler, directory=web)
        try:
            self._httpd = http.server.ThreadingHTTPServer(("0.0.0.0", port), handler)
        except OSError as e:
            self.get_logger().warn(f"HTTP-Server :{port} nicht startbar: {e}")
            return
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.get_logger().info(
            f"Hand-GUIs: http://localhost:{port}/hand_controller_viewer.html | "
            f"http://localhost:{port}/inspire_hand_viewer.html")

    # ── ROS-seitig ────────────────────────────────────────────────────────────
    def _step(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        self.backend.update(self.models, dt)

    def _publish_joint_states(self, names, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(p) for p in positions]
        self.joint_pub.publish(msg)

    # ── Controller-WebSocket (:8766) ───────────────────────────────────────────
    async def _controller_handler(self, ws):
        self.get_logger().info("Controller-GUI verbunden")
        await ws.send(json.dumps({
            "type": "config",
            "dof_names": joint_map.DOF_NAMES,
            "left":  self.models["left"].controller_state(),
            "right": self.models["right"].controller_state(),
        }))
        try:
            async for raw in ws:
                try:
                    self._handle_cmd(json.loads(raw))
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f"CMD-Fehler: {e}")
        except websockets.exceptions.ConnectionClosed:
            pass

    def _handle_cmd(self, cmd: dict):
        side = cmd.get("side", "left")
        hand = self.models["right"] if side == "right" else self.models["left"]
        t = cmd.get("type")
        if t == "set_angle":
            hand.set_angle(cmd["dof"], cmd["value"])
        elif t == "set_force":
            hand.set_force(cmd["dof"], cmd["value"])
        elif t == "set_speed":
            # Beide Haende gleich schnell (wie im Original).
            self.models["left"].set_speed_all(cmd["value"])
            self.models["right"].set_speed_all(cmd["value"])
        elif t == "set_enabled":
            hand.set_enabled(cmd["value"])
        elif t == "set_all_angles":
            for i, v in enumerate(cmd["values"]):
                hand.set_angle(i, v)
        elif t == "open_hand":
            self._apply_open_close(hand, "open")
        elif t == "close_hand":
            self._apply_open_close(hand, "close")

    @staticmethod
    def _apply_open_close(hand: HandModel, action: str):
        """Alle 6 DOF auf offen (1000) bzw. zu setzen. Gemeinsam genutzt von der
        Controller-GUI (open_hand/close_hand) und den Streamdeck-Hand-Buttons."""
        hand.set_enabled(True)
        if action == "open":
            for i in range(6):
                hand.set_angle(i, 1000)
        else:  # close
            for i in range(6):
                hand.set_angle(i, 200 if i == 4 else 0)  # Daumen-Beugung nicht ganz zu

    def _on_hand_action(self, side: str, msg: String):
        """Streamdeck OPEN/CLOSE {LEFT,RIGHT} HAND -> Inspire-DOF-Sollwerte."""
        action = (msg.data or "").strip().lower()
        if action not in ("open", "close"):
            return
        hand = self.models["right"] if side == "right" else self.models["left"]
        self._apply_open_close(hand, action)
        self.get_logger().info(f"Streamdeck: Hand {side} -> {action}")

    async def _controller_broadcast(self, clients):
        while True:
            if clients:
                msg = json.dumps({
                    "type": "state",
                    "left":  self.models["left"].controller_state(),
                    "right": self.models["right"].controller_state(),
                })
                await _send_all(clients, msg)
            await asyncio.sleep(0.05)

    # ── Viewer-WebSocket (:8765) ───────────────────────────────────────────────
    async def _viewer_handler(self, ws):
        self.get_logger().info("Viewer-GUI verbunden")
        m = tactile.meta()
        m.update({"type": "meta", "dof_names": joint_map.DOF_NAMES})
        try:
            await ws.send(json.dumps(m))
            async for _ in ws:
                pass
        except websockets.exceptions.ConnectionClosed:
            pass

    async def _viewer_broadcast(self, clients):
        while True:
            if clients:
                msg = json.dumps({
                    "type": "data",
                    "left":  self.models["left"].viewer_state(),
                    "right": self.models["right"].viewer_state(),
                })
                await _send_all(clients, msg)
            await asyncio.sleep(0.05)

    # ── asyncio-Hauptschleife ──────────────────────────────────────────────────
    async def run_ws(self):
        ctrl_clients: set = set()
        view_clients: set = set()

        async def ctrl(ws):
            ctrl_clients.add(ws)
            try:
                await self._controller_handler(ws)
            finally:
                ctrl_clients.discard(ws)

        async def view(ws):
            view_clients.add(ws)
            try:
                await self._viewer_handler(ws)
            finally:
                view_clients.discard(ws)

        async with websockets.serve(ctrl, self.ws_host, self.controller_port), \
                   websockets.serve(view, self.ws_host, self.viewer_port):
            await asyncio.gather(
                self._controller_broadcast(ctrl_clients),
                self._viewer_broadcast(view_clients),
            )


class _QuietHttpHandler(http.server.SimpleHTTPRequestHandler):
    """SimpleHTTPRequestHandler ohne Zeile-pro-Request-Logging."""

    def log_message(self, fmt, *args):  # noqa: D102
        pass


async def _send_all(clients: set, msg: str):
    dead = set()
    for ws in list(clients):
        try:
            await ws.send(msg)
        except Exception:  # noqa: BLE001
            dead.add(ws)
    clients -= dead


def main(args=None):
    rclpy.init(args=args)
    node = InspireFtpBridge()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        asyncio.run(node.run_ws())
    except KeyboardInterrupt:
        pass
    finally:
        node.backend.shutdown()
        if node._httpd is not None:
            node._httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
