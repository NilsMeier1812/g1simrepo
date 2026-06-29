#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
inspire_hand — Bruecke ROS2 <-> Sim-DDS fuer die Inspire-FTP-Finger (pragmatisch).

GUI/Tools schicken Finger-Ziele (ROS2). Dieser Node setzt sie als DDS rt/inspire/cmd
(LowCmd_) -> die MuJoCo-Bridge faehrt die Finger-Position-Servos. Den Sim-Ist-Zustand
(rt/inspire/state, LowState_) publiziert er als Finger-/joint_states -> RViz bewegt die
Finger mit, und als /g1pilot/inspire/state (JointState) fuer die GUI.

Pragmatischer GUI-Topic; spaeter auf das echte Inspire-DDS-Format hebbar.

Kanonische Finger-Reihenfolge (identisch in MJCF-Generator, Bridge, robot_state):
  je Hand: thumb 1..4, index 1..2, middle 1..2, ring 1..2, little 1..2  (12),
  links dann rechts -> 24 Slots. cmd/state-Slot k <-> FINGER_JOINTS[k].

ROS2-Schnittstelle:
  Sub /g1pilot/inspire/cmd  (sensor_msgs/JointState)         : Ziel-Winkel [rad] by name
  Sub /g1pilot/inspire/grip (std_msgs/Float64MultiArray [l,r]): 0=offen .. 1=zu (Komfort)
  Pub /joint_states         (sensor_msgs/JointState)         : Finger-Ist -> RViz
  Pub /g1pilot/inspire/state(sensor_msgs/JointState)         : Finger-Ist (GUI)

WICHTIG: Laeuft dieser Node, gehoeren die Finger IHM. robot_state dann mit
publish_hand_joints:=false starten, sonst ueberschreiben sich die /joint_states.

Aufruf:  ros2 run g1pilot inspire_hand --ros-args -p interface:=lo
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

from g1pilot.utils.common import init_dds


FINGER_JOINTS = [
    f"{side}_{finger}_{i}_joint"
    for side in ("left", "right")
    for finger, n in (("thumb", 4), ("index", 2), ("middle", 2), ("ring", 2), ("little", 2))
    for i in range(1, n + 1)
]
NJ = len(FINGER_JOINTS)  # 24


class InspireHand(Node):
    def __init__(self):
        super().__init__("inspire_hand")

        self.declare_parameter("interface", "lo")
        self.declare_parameter("rate_hz", 50.0)
        # Default-Schliesswinkel je Gelenk, falls die URDF-Limits nicht gelesen werden
        # koennen (nur fuer die /grip-Komfortfunktion; explizite cmd-Ziele unabhaengig).
        self.declare_parameter("urdf", "g1_29dof_inspire_ftp.urdf")

        interface = self.get_parameter("interface").get_parameter_value().string_value
        rate = float(self.get_parameter("rate_hz").value)

        # Gelenk-Limits (lo, hi) je Finger fuer /grip + Clamping. Aus der installierten
        # URDF gelesen; faellt sonst auf [0, 1.4] zurueck.
        self.lo = np.zeros(NJ, dtype=float)
        self.hi = np.full(NJ, 1.4, dtype=float)
        self._load_limits()

        self.idx = {name: k for k, name in enumerate(FINGER_JOINTS)}
        self.targets = self.lo.copy()      # Start: offene Hand
        self.state_q = np.zeros(NJ, dtype=float)

        init_dds(interface, self.get_logger())
        self.cmd_pub = ChannelPublisher("rt/inspire/cmd", LowCmd_)
        self.cmd_pub.Init()
        self.cmd_msg = unitree_hg_msg_dds__LowCmd_()
        self.state_sub = ChannelSubscriber("rt/inspire/state", LowState_)
        self.state_sub.Init(self._on_dds_state, 10)

        self.create_subscription(JointState, "/g1pilot/inspire/cmd", self._on_cmd, 10)
        self.create_subscription(Float64MultiArray, "/g1pilot/inspire/grip", self._on_grip, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.state_pub = self.create_publisher(JointState, "/g1pilot/inspire/state", 10)

        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"inspire_hand bereit ({NJ} Finger). cmd: /g1pilot/inspire/cmd (JointState) "
            f"oder /g1pilot/inspire/grip ([links,rechts] 0..1). interface={interface}.")

    def _load_limits(self):
        try:
            import xml.etree.ElementTree as ET
            share = get_package_share_directory("g1pilot")
            urdf = os.path.join(share, "description_files", "urdf",
                                self.get_parameter("urdf").get_parameter_value().string_value)
            root = ET.parse(urdf).getroot()
            lim = {}
            for j in root.findall("joint"):
                if j.get("name") in FINGER_JOINTS:
                    l = j.find("limit")
                    if l is not None:
                        lim[j.get("name")] = (float(l.get("lower", 0.0)), float(l.get("upper", 1.4)))
            for k, name in enumerate(FINGER_JOINTS):
                if name in lim:
                    self.lo[k], self.hi[k] = lim[name]
            self.get_logger().info(f"Finger-Limits aus URDF geladen ({len(lim)}/{NJ}).")
        except Exception as e:
            self.get_logger().warn(f"URDF-Limits nicht ladbar ({e}) -> Default [0,1.4].")

    # ── ROS2 → Ziele ────────────────────────────────────────────────────────
    def _on_cmd(self, msg: JointState):
        for name, q in zip(msg.name, msg.position):
            k = self.idx.get(name)
            if k is not None:
                self.targets[k] = float(np.clip(q, self.lo[k], self.hi[k]))

    def _on_grip(self, msg: Float64MultiArray):
        d = list(msg.data)
        if not d:
            return
        fl = float(np.clip(d[0], 0.0, 1.0))
        fr = float(np.clip(d[1] if len(d) > 1 else d[0], 0.0, 1.0))
        for k in range(NJ):
            frac = fl if k < NJ // 2 else fr      # 0..11 links, 12..23 rechts
            self.targets[k] = self.lo[k] + frac * (self.hi[k] - self.lo[k])

    # ── DDS State → /joint_states ───────────────────────────────────────────
    def _on_dds_state(self, msg: LowState_):
        for k in range(NJ):
            self.state_q[k] = float(msg.motor_state[k].q)

    def _tick(self):
        # Ziele -> DDS cmd (Bridge faehrt die Position-Servos)
        for k in range(NJ):
            self.cmd_msg.motor_cmd[k].q = float(self.targets[k])
        self.cmd_pub.Write(self.cmd_msg)
        # Ist -> /joint_states (RViz) + GUI-State
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = FINGER_JOINTS
        js.position = [float(x) for x in self.state_q]
        self.js_pub.publish(js)
        self.state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = InspireHand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
