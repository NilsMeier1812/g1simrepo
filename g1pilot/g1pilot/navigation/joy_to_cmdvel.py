#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joy_to_cmdvel — Sim-Bruecke: /g1pilot/joy (Joy) -> /g1pilot/loco_cmd_vel (Twist).

Der g1pilot-Nav-Stack faehrt den Roboter ueber einen "virtuellen Joystick":
nav2point -> /g1pilot/auto_joy -> joy_mux -> /g1pilot/joy. Auf dem ECHTEN G1
liest loco_client diesen Joy direkt (joystick_callback -> Move()). In der SIM
laeuft aber loco_sim, das KEINEN Joy liest, sondern nur die normierte
Geschwindigkeit /g1pilot/loco_cmd_vel (Twist).

Diese Node uebersetzt daher in der Sim das (bereits durch joy_mux/auto_enable
gegatete) /g1pilot/joy in /g1pilot/loco_cmd_vel -- so faehrt derselbe Nav-Stack
in der Sim wie auf dem echten Roboter. Nur fuer die Sim gedacht (bringup_sim bei
G1_ENABLE_NAV=1). Der Real-Pfad braucht sie NICHT.

Achsen-Konvention (identisch zu nav2point/loco_client):
  physisch vorwaerts vx  =  -axes[1] * max_vx   -> normiert  linear.x  = -axes[1]
  physisch seit    vy    =  -axes[0] * max_vy   -> normiert  linear.y  = -axes[0]
  physisch dreh   vyaw   =  -axes[2] * max_vyaw -> normiert  angular.z = -axes[2]
loco_sim skaliert die normierten [-1,1]-Werte selbst auf m/s bzw. rad/s.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def _clamp(v):
    return max(-1.0, min(1.0, v))


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmdvel')
        self.declare_parameter('joy_topic', '/g1pilot/joy')
        self.declare_parameter('cmd_vel_topic', '/g1pilot/loco_cmd_vel')
        # Watchdog: bleibt der Joy-Stream aus (Nav aus, joy_mux/nav2point tot),
        # EINMAL 0 senden -> loco_sim stoppt. Sonst bliebe das letzte Kommando
        # stehen (loco_sim hat keinen eigenen cmd-Timeout).
        self.declare_parameter('hold_timeout', 0.3)
        joy_topic = self.get_parameter('joy_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.hold_timeout = float(self.get_parameter('hold_timeout').value)

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, cmd_topic, qos)
        self.create_subscription(Joy, joy_topic, self._on_joy, qos)
        self._last_rx = None
        self._stopped_sent = True
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            "joy_to_cmdvel (Sim): %s -> %s" % (joy_topic, cmd_topic))

    def _on_joy(self, msg: Joy):
        ax = msg.axes
        # Fehlende Achsen tolerieren (defensive: manche Joy-Quellen liefern < 3).
        a0 = ax[0] if len(ax) > 0 else 0.0
        a1 = ax[1] if len(ax) > 1 else 0.0
        a2 = ax[2] if len(ax) > 2 else 0.0
        t = Twist()
        t.linear.x = _clamp(-a1)
        t.linear.y = _clamp(-a0)
        t.angular.z = _clamp(-a2)
        self.pub.publish(t)
        self._last_rx = time.monotonic()
        self._stopped_sent = False

    def _watchdog(self):
        if self._last_rx is None or self._stopped_sent:
            return
        if (time.monotonic() - self._last_rx) > self.hold_timeout:
            self.pub.publish(Twist())   # alle Felder 0 -> Stopp
            self._stopped_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
