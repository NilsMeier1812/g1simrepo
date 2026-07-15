#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_localization — Sim-Lokalisierung (Ersatz fuer MOLA in der MuJoCo-Sim).

Auf dem echten G1 liefert MOLA (Livox-SLAM) die Roboter-Pose auf
`/lidar_odometry/pose_fixed`. In der Sim gibt es kein LiDAR/MOLA -- diese Node
tritt an dieselbe Stelle: sie liest die Ground-Truth-Basispose aus der MuJoCo-
Bridge (`rt/sportmodestate`: Basis-x/y/z + -Velocity) und die Basis-Orientierung
aus `rt/lowstate` (IMU-Quaternion) und publiziert daraus EXAKT die Topics, die
der g1pilot-Nav-Stack (dijkstra_planner, nav2point, joy_mux) erwartet:

  /lidar_odometry/pose_fixed  (nav_msgs/Odometry, Frame 'map')  ← Pose fuer Planer/Follower
  TF  odom_unitree -> base_link (dynamisch)                     ← RViz zeigt den Roboter bewegt

Damit laeuft der KOMPLETTE, unveraenderte Real-Nav-Stack in der Sim -- der
einzige Unterschied ist die Pose-Quelle (hier statt MOLA). So kann eine
Navigation gefahrlos in MuJoCo validiert werden, bevor sie an den echten G1 geht.

Nur fuer die Sim gedacht (wird von bringup_sim bei G1_ENABLE_NAV=1 gestartet).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

from g1pilot.utils.common import init_dds


class SimLocalization(Node):
    def __init__(self):
        super().__init__('sim_localization')

        self.declare_parameter('interface', 'lo')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom_unitree')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        qos = QoSProfile(depth=10)
        self.pub_odom = self.create_publisher(Odometry, '/lidar_odometry/pose_fixed', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Letzte Basis-Orientierung (IMU-Quaternion w,x,y,z) aus rt/lowstate.
        self._quat = (1.0, 0.0, 0.0, 0.0)

        init_dds(interface, self.get_logger())
        # IMU-Orientierung: aus rt/lowstate (die MuJoCo-Bridge fuellt sportmodestate
        # NICHT mit Orientierung, nur mit Position/Velocity).
        self.sub_low = ChannelSubscriber('rt/lowstate', LowState_)
        self.sub_low.Init(self._on_lowstate)
        # Basis-Position + -Velocity: aus rt/sportmodestate (MuJoCo-Ground-Truth).
        self.sub_high = ChannelSubscriber('rt/sportmodestate', SportModeState_)
        self.sub_high.Init(self._on_highstate)

        self.get_logger().info(
            "sim_localization aktiv: rt/sportmodestate (+IMU) -> "
            "/lidar_odometry/pose_fixed (Frame '%s')." % self.map_frame)

    def _on_lowstate(self, msg: LowState_):
        q = msg.imu_state.quaternion  # [w, x, y, z]
        self._quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    def _on_highstate(self, msg: SportModeState_):
        now = self.get_clock().now().to_msg()
        px, py, pz = float(msg.position[0]), float(msg.position[1]), float(msg.position[2])
        vx, vy, vz = float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])
        qw, qx, qy, qz = self._quat

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = px
        odom.pose.pose.position.y = py
        odom.pose.pose.position.z = pz
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self.pub_odom.publish(odom)

        # Dynamische TF odom_unitree -> base_link, damit RViz den Roboter in der
        # Karte bewegt zeigt (bringup_sim laesst bei Nav die statische Identity-TF
        # dieser Kante weg -> hier ist die einzige Quelle, kein Doppel-Parent).
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = px
            t.transform.translation.y = py
            t.transform.translation.z = pz
            t.transform.rotation.w = qw
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
