#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scene_bridge — empfaengt die Umgebungs-Objekte (Hindernisse + greifbare
Objekte) vom MuJoCo-Container per UDP (siehe unitree_mujoco/simulate_python/
scene_state_publisher.py, der Sender) und veroeffentlicht sie als
visualization_msgs/MarkerArray auf /scene_markers.

/scene_markers ist das EINE geteilte Weltmodell (siehe g1pilot/docs/51_navigation_technik.md):
  - RViz zeigt es direkt an (MarkerArray-Display in nav.rviz).
  - create_map.py rastert daraus die 2D-Hindernis-/Nav-Karte (/map).
  - arm_controller.py/ik_solver.py speisen daraus die Umgebungs-Kollision
    (Hindernis -> ausweichen, Greif-Objekt -> Hand darf ran).

Warum UDP statt eines ROS-Topics auf der Sim-Seite? Der MuJoCo-Container hat
KEIN ROS (siehe push_listener.py/grasp_box.py fuer dasselbe Muster in der
Gegenrichtung). Beide Container laufen mit network_mode: host -> Loopback
verbindet sie ohne DDS/ROS.

Grosse Umgebungen (CAD-Zellen mit Hunderten Teilen) kommen in mehreren
Datagrammen an; scene_protocol.SnapshotAssembler setzt sie zusammen.
Veroeffentlicht wird nur, wenn sich etwas geaendert hat (plus ein Keepalive
alle `keepalive_s` Sekunden) - sonst rechnen IK und Nav-Karte bei 700 Teilen
10x pro Sekunde dieselben Marker um.
"""
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from g1pilot.navigation import scene_markers as sm
from g1pilot.navigation.scene_protocol import SnapshotAssembler
from g1pilot.utils import lowpoly


class SceneBridge(Node):
    def __init__(self):
        super().__init__("scene_bridge")

        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 47902)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_id", "map")
        # Fixer, gut bekannter In-Container-Pfad fuer die Mesh-Dateien (siehe
        # docker-compose.yml: unitree_mujoco/scene_editor wird read-only genau
        # hierhin gemountet; die Sim schickt Pfade relativ dazu, z.B.
        # scenes/<name>/meshes/teil.stl). RViz laedt MESH_RESOURCE-Marker darueber.
        self.declare_parameter("mesh_resource_prefix", "file:///scene_editor/")
        # Auch ohne Aenderung spaetestens so oft neu veroeffentlichen (s).
        self.declare_parameter("keepalive_s", 2.0)

        self.udp_host = str(self.get_parameter("udp_host").value)
        self.udp_port = int(self.get_parameter("udp_port").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.mesh_prefix = str(self.get_parameter("mesh_resource_prefix").value)
        self.keepalive_s = float(self.get_parameter("keepalive_s").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_markers = self.create_publisher(MarkerArray, "/scene_markers", qos)

        self._lock = threading.Lock()
        self._assembler = SnapshotAssembler()
        self._last_rx_time = 0.0
        self._published_version = -1
        self._last_publish_time = 0.0
        self._published_ids = set()  # fuer sauberes DELETE verschwundener Objekte

        # Sparsame Grafik (G1_LOW_GFX=1): RViz zeigt vereinfachte Mesh-Kopien.
        # Nur mesh_resource aendert sich -- IK und Nav-Karte lesen ausschliesslich
        # Pose/Scale/AABB (scene_markers.py) und sehen exakt dieselben Marker.
        self._low_gfx = lowpoly.enabled()
        self._lowpoly_uri = {}       # (mesh, scale) -> URI fuer RViz

        self._sock = None
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

        self.timer = self.create_timer(1.0 / rate if rate > 0 else 0.1, self._publish)

        self.get_logger().info(
            f"scene_bridge aktiv: UDP {self.udp_host}:{self.udp_port} -> "
            f"/scene_markers (Frame '{self.frame_id}')."
        )

    # ── UDP-Empfangs-Thread ──────────────────────────────────────────────
    def _listen(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.udp_host, self.udp_port))
        except OSError as e:
            self.get_logger().error(
                f"UDP-Port {self.udp_port} konnte nicht gebunden werden: {e} -- "
                f"scene_bridge bleibt ohne Szenen-Daten (RViz/Nav/IK sehen keine "
                f"Objekte)."
            )
            return
        while rclpy.ok():
            try:
                data, _ = self._sock.recvfrom(65536)
            except OSError:
                break
            with self._lock:
                self._assembler.feed(data)
                self._last_rx_time = time.time()

    # ── Marker-Aufbau ────────────────────────────────────────────────────
    def _obj_to_marker(self, obj: dict) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = sm.ns_for_class(obj.get("class", "obstacle"))
        m.id = sm.stable_id(obj["name"])
        m.type = sm.marker_type_for(obj.get("type", "box"))
        m.action = Marker.ADD

        pos = obj.get("pos", [0.0, 0.0, 0.0])
        quat = obj.get("quat", [1.0, 0.0, 0.0, 0.0])   # w,x,y,z
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (
            float(pos[0]), float(pos[1]), float(pos[2]))
        m.pose.orientation.w = float(quat[0])
        m.pose.orientation.x = float(quat[1])
        m.pose.orientation.y = float(quat[2])
        m.pose.orientation.z = float(quat[3])

        size = obj.get("size")
        sx, sy, sz = sm.marker_scale_for(obj.get("type", "box"), size)
        m.scale.x, m.scale.y, m.scale.z = sx, sy, sz

        rgba = obj.get("rgba", [0.7, 0.7, 0.7, 1.0])
        m.color.r, m.color.g, m.color.b, m.color.a = (
            float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3]))

        if obj.get("type") == "mesh" and obj.get("mesh"):
            m.mesh_resource = self._mesh_uri(str(obj["mesh"]), sx)
            m.mesh_use_embedded_materials = False

        m.text = sm.encode_text(obj["name"], obj.get("aabb_half"))
        return m

    def _mesh_uri(self, mesh: str, scale: float) -> str:
        uri = self.mesh_prefix + mesh
        if not self._low_gfx or not uri.startswith("file://"):
            return uri
        key = (mesh, scale)
        if key not in self._lowpoly_uri:
            out = None
            try:
                out = lowpoly.lowpoly_file(uri[len("file://"):], lowpoly.ENV_CELL_M,
                                           scale=scale, subdir="scene")
            except Exception as e:   # Anzeige-Optimierung, nie ein harter Fehler
                self.get_logger().warn(f"LOW_GFX: '{mesh}' bleibt original ({e}).")
            self._lowpoly_uri[key] = ("file://" + out) if out else uri
        return self._lowpoly_uri[key]

    def _publish(self):
        with self._lock:
            if not self._assembler.ready():
                return
            version = self._assembler.version
            now = time.time()
            if (version == self._published_version
                    and now - self._last_publish_time < self.keepalive_s):
                return
            objs = self._assembler.objects()
        self._published_version = version
        self._last_publish_time = now

        array = MarkerArray()
        seen_ids = set()
        for obj in objs:
            name = obj.get("name")
            if not name:
                continue
            try:
                marker = self._obj_to_marker(obj)
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().warn(f"Ungueltiges Szenen-Objekt '{name}' uebersprungen: {e}")
                continue
            array.markers.append(marker)
            seen_ids.add((marker.ns, marker.id))

        # Objekte, die im letzten Snapshot noch da waren und jetzt fehlen
        # (Szene gewechselt/Objekt entfernt): sauber aus RViz loeschen.
        for ns, mid in self._published_ids - seen_ids:
            gone = Marker()
            gone.header.frame_id = self.frame_id
            gone.header.stamp = self.get_clock().now().to_msg()
            gone.ns = ns
            gone.id = mid
            gone.action = Marker.DELETE
            array.markers.append(gone)
        self._published_ids = seen_ids

        if array.markers:
            self.pub_markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SceneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
