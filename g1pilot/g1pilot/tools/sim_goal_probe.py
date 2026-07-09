#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_goal_probe — Diagnose fuer die Arm-Steuerkette OHNE Interactive Marker.

Zweck
-----
Isoliert testen, ob der arm_controller auf ein Hand-Ziel reagiert:
  Ziel (PoseStamped)  ->  arm_controller  ->  IK  ->  rt/arm_sdk  ->  MuJoCo
Der Interactive Marker wird dabei komplett umgangen. Wenn sich der Arm mit
diesem Skript bewegt, lag das Problem nur am (stummen, grauen) Marker.
Bewegt er sich NICHT, liegt es am Controller/IK und das Log zeigt, wo.

Vorgehen
--------
1. Liest per TF die aktuelle Hand-Pose (frame -> ee_frame).
2. Aktiviert die Arme (/g1pilot/arms/enabled = true), sofern --no-enable nicht.
3. Publiziert mit ~20 Hz ein Ziel auf /g1pilot/hand_goal/<side>, das auf einer
   Achse sinusfoermig um die Startpose schwingt (Default z, +/-6 cm).
   (Der erste Sample wird vom arm_controller zur Auto-Kalibrierung benutzt –
   deshalb oszillieren wir, damit danach echte Bewegung entsteht.)
4. Loggt 1x/s das kommandierte Offset und die gemessenen Arm-Gelenkwinkel aus
   /joint_states, sodass man Soll vs. Ist direkt vergleichen kann.

Aufruf (im g1pilot-Container, nach source):
  ros2 run g1pilot sim_goal_probe
  ros2 run g1pilot sim_goal_probe --ros-args -p side:=left -p axis:=x -p amp:=0.08
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import rclpy.time


# Gelenke, die wir zur Kontrolle der Bewegung mitloggen (ROS-Namen aus URDF).
WATCH_JOINTS = {
    "right": ["right_shoulder_pitch_joint", "right_elbow_joint", "right_wrist_pitch_joint"],
    "left":  ["left_shoulder_pitch_joint",  "left_elbow_joint",  "left_wrist_pitch_joint"],
}


class SimGoalProbe(Node):
    def __init__(self):
        super().__init__("sim_goal_probe")

        self.declare_parameter("side", "right")          # right | left
        self.declare_parameter("frame", "pelvis")        # ik_world_frame des arm_controllers
        self.declare_parameter("axis", "z")              # x | y | z
        self.declare_parameter("amp", 0.06)              # Amplitude [m]
        self.declare_parameter("period", 4.0)            # Schwingungsdauer [s]
        self.declare_parameter("rate", 20.0)             # Publish-Rate [Hz]
        self.declare_parameter("auto_enable", True)      # Arme automatisch enablen

        self.side = self.get_parameter("side").value.lower()
        self.frame = self.get_parameter("frame").value
        self.axis = self.get_parameter("axis").value.lower()
        self.amp = float(self.get_parameter("amp").value)
        self.period = float(self.get_parameter("period").value)
        self.rate = float(self.get_parameter("rate").value)
        self.auto_enable = bool(self.get_parameter("auto_enable").value)

        if self.side not in ("right", "left"):
            self.get_logger().error(f"side muss right|left sein, nicht '{self.side}'")
            raise SystemExit(1)
        self.ee_frame = f"{self.side}_hand_point_contact"
        self.axis_idx = {"x": 0, "y": 1, "z": 2}[self.axis]
        self.goal_topic = f"/g1pilot/hand_goal/{self.side}"

        qos = QoSProfile(depth=10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, qos)
        self.enable_pub = self.create_publisher(Bool, "/g1pilot/arms/enabled", qos)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.base = None              # (x, y, z, qx, qy, qz, qw)
        self.last_joints = {}
        self.t0 = None
        self._log_accum = 0.0
        self._enable_sent = 0

        self.get_logger().info(
            f"[probe] side={self.side} topic={self.goal_topic} ee_frame={self.ee_frame} "
            f"axis={self.axis} amp={self.amp:.3f}m period={self.period:.1f}s rate={self.rate:.0f}Hz"
        )
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

    def _on_joints(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.last_joints[n] = p

    def _capture_base(self) -> bool:
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, self.ee_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False
        t = tf.transform.translation
        r = tf.transform.rotation
        self.base = (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        self.get_logger().info(
            f"[probe] Start-Pose ({self.frame}->{self.ee_frame}): "
            f"pos=({t.x:.3f},{t.y:.3f},{t.z:.3f})"
        )
        return True

    def _tick(self):
        # 1) Arme aktivieren (ein paar Mal, bis Subscriber sicher verbunden ist)
        if self.auto_enable and self._enable_sent < 5:
            self.enable_pub.publish(Bool(data=True))
            self._enable_sent += 1

        # 2) Startpose aus TF holen (einmalig)
        if self.base is None:
            if not self._capture_base():
                self.get_logger().info(
                    f"[probe] warte auf TF {self.frame} -> {self.ee_frame} ...",
                    throttle_duration_sec=1.0)
                return
            self.t0 = self.get_clock().now()

        # 3) Sinus-Ziel publizieren
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        offset = self.amp * math.sin(2.0 * math.pi * t / self.period)

        ps = PoseStamped()
        ps.header.frame_id = self.frame
        ps.header.stamp = self.get_clock().now().to_msg()
        bx, by, bz, qx, qy, qz, qw = self.base
        pos = [bx, by, bz]
        pos[self.axis_idx] += offset
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pos
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.goal_pub.publish(ps)

        # 4) 1x/s Soll vs. Ist loggen
        self._log_accum += 1.0 / self.rate
        if self._log_accum >= 1.0:
            self._log_accum = 0.0
            watch = WATCH_JOINTS[self.side]
            meas = ", ".join(
                f"{n.replace(f'{self.side}_', '').replace('_joint', '')}="
                f"{self.last_joints.get(n, float('nan')):+.3f}"
                for n in watch
            )
            have_js = "ja" if self.last_joints else "NEIN (kein /joint_states!)"
            self.get_logger().info(
                f"[probe] cmd {self.axis}-offset={offset:+.3f}m | js={have_js} | {meas}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimGoalProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
