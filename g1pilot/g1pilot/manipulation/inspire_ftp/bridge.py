#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bridge.py — Inspire-FTP-Hand-Bridge fuer die G1-Sim
===================================================
Vereint den frueheren ftp_hand_controller (zwei eigenstaendige Python-Skripte,
die per Modbus TCP mit den echten Haenden sprachen) in EINEM ROS2-Node, der
stattdessen mit der Simulation redet:

  * WebSocket :8766  <-> Controller/hand_controller_viewer.html  (Steuerung)
  * WebSocket :8765  <-> Viewer/inspire_hand_viewer.html         (Kraft/Taktil)
  * ROS2 /joint_states                                            (RViz-Finger)

Die eigentliche "Hardware"-Anbindung steckt im Backend (siehe backends.py):
Stufe 1 = SimJointStateBackend (RViz). Beide GUIs funktionieren unveraendert.

WICHTIG: Wenn dieser Node laeuft, muss robot_state mit publish_hand_joints:=false
gestartet werden, sonst ueberschreiben sich die beiden /joint_states-Quellen fuer
die Finger gegenseitig (so auch im robot_state.py-Kommentar vermerkt).
"""

from __future__ import annotations

import asyncio
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

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
        self.declare_parameter("update_rate_hz", 50.0)
        # backend: "mujoco" = Stufe 2 (echte Finger-Physik + Touch-Kraefte via DDS),
        #          "sim"    = Stufe 1 (open-loop /joint_states, Kraft/Taktil = 0).
        self.declare_parameter("backend", "mujoco")
        self.declare_parameter("interface", "lo")        # DDS-Interface fuer 'mujoco'
        # Nur kosmetisch fuer die GUI-Anzeige (kein echtes Modbus-Ziel im Sim).
        self.declare_parameter("left_host", "sim://left")
        self.declare_parameter("right_host", "sim://right")

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
        if backend_name == "mujoco":
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

        # ── Update-Schleife (rclpy-Timer): Backend treiben + /joint_states ───
        self._last = time.monotonic()
        self.create_timer(1.0 / max(rate, 1.0), self._step)

        self.get_logger().info(
            f"Inspire-FTP-Bridge aktiv | Controller ws://{self.ws_host}:{self.controller_port} | "
            f"Viewer ws://{self.ws_host}:{self.viewer_port} | Backend={backend_name}"
        )

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
            for i in range(6):
                hand.set_angle(i, 1000)
        elif t == "close_hand":
            for i in range(6):
                hand.set_angle(i, 200 if i == 4 else 0)  # Daumen-Beugung nicht ganz zu

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
