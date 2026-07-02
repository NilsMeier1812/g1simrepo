#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import subprocess
import sys
import threading
import time
from PyQt6.QtWidgets import (
    QApplication, QWidget, QGridLayout, QPushButton, QVBoxLayout,
    QHBoxLayout, QSlider, QLabel
)
from PyQt6.QtCore import QTimer, Qt, QPointF
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from g1pilot.utils.window_style import DarkStyle


class VirtualJoystick(QWidget):
    """Bildschirm-Joystick: mit der Maus ziehen -> normierte (vx, vy) in [-1,1].

    Oben = vorwaerts (+vx), links = +vy (Roboter-Koordinaten). Loslassen zentriert
    auf (0,0). loco_sim multipliziert mit max_cmd -> vx=1.0 entspricht ~0.8 m/s.
    Wird im RUN-Zustand als Geschwindigkeitsbefehl ausgewertet; zentrieren loest den
    automatischen Handoff zurueck zum PD-Stand aus."""

    def __init__(self, size=180):
        super().__init__()
        self._size = size
        self.setFixedSize(size, size)
        self._radius = size / 2 - 16
        self._knob = QPointF(0.0, 0.0)   # Offset vom Zentrum (Pixel)
        self.vx = 0.0
        self.vy = 0.0

    def _update_vel(self):
        r = self._radius
        # Bildschirm-y zeigt nach unten -> oben (negatives dy) = vorwaerts.
        self.vx = max(-1.0, min(1.0, -self._knob.y() / r))
        self.vy = max(-1.0, min(1.0, -self._knob.x() / r))

    def _set_from_pos(self, pos):
        c = self._size / 2
        dx = pos.x() - c
        dy = pos.y() - c
        d = math.hypot(dx, dy)
        if d > self._radius and d > 0:
            dx *= self._radius / d
            dy *= self._radius / d
        self._knob = QPointF(dx, dy)
        self._update_vel()
        self.update()

    def mousePressEvent(self, e):
        self._set_from_pos(e.position())

    def mouseMoveEvent(self, e):
        self._set_from_pos(e.position())

    def mouseReleaseEvent(self, e):
        self._knob = QPointF(0.0, 0.0)
        self.vx = self.vy = 0.0
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = self._size / 2
        p.setBrush(QBrush(QColor("#1e1e1e")))
        p.setPen(QPen(QColor("#444"), 2))
        p.drawEllipse(QPointF(c, c), self._radius + 10, self._radius + 10)
        p.setPen(QPen(QColor("#333"), 1))
        p.drawLine(int(c - self._radius), int(c), int(c + self._radius), int(c))
        p.drawLine(int(c), int(c - self._radius), int(c), int(c + self._radius))
        active = abs(self.vx) > 1e-3 or abs(self.vy) > 1e-3
        p.setBrush(QBrush(QColor("#4CAF50") if active else QColor("#3c3c3c")))
        p.setPen(QPen(QColor("#80ff80") if active else QColor("#666"), 2))
        p.drawEllipse(QPointF(c + self._knob.x(), c + self._knob.y()), 15, 15)


class StreamDeck(Node):
    def __init__(self):
        super().__init__('stream_deck')

        self.pub_start = self.create_publisher(Bool, '/g1pilot/start', 10)
        self.pub_start_balancing = self.create_publisher(Bool, '/g1pilot/start_balancing', 10)
        self.pub_start_walking = self.create_publisher(Bool, '/g1pilot/start_walking', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/g1pilot/loco_cmd_vel', 10)
        self.pub_arms_enabled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 10)
        self.pub_arms_home = self.create_publisher(Bool, '/g1pilot/arms/home', 10)
        self.pub_marker_follow = self.create_publisher(Bool, '/g1pilot/marker_follow_ee', 10)
        self.pub_left_hand = self.create_publisher(String, '/g1pilot/hand_action/left', 10)
        self.pub_right_hand = self.create_publisher(String, '/g1pilot/hand_action/right', 10)
        self.pub_emergency_stop = self.create_publisher(Bool, '/g1pilot/emergency_stop', 10)
        self.pub_push = self.create_publisher(Bool, '/g1pilot/push', 10)
        self.pub_catch_falls = self.create_publisher(Bool, '/g1pilot/catch_falls', 10)

    def publish_bool(self, pub, value: bool):
        msg = Bool()
        msg.data = value
        pub.publish(msg)

    def publish_str(self, pub, text: str):
        msg = String()
        msg.data = text
        pub.publish(msg)


class ButtonGUI(QWidget):
    """PyQt6 GUI for ROS2 StreamDeck with timed and emergency behaviors."""

    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.button_states = {}
        self.hand_pairs = {
            "left": {"open": (2, 0), "close": (2, 1)},
            "right": {"open": (2, 2), "close": (2, 3)},
        }

        self.setWindowTitle("DIGITAL STREAMDECK")
        self.init_ui()
        self.apply_style()

        # WALK-Geschwindigkeit kontinuierlich (~30 Hz) publizieren. loco_sim wertet
        # /g1pilot/loco_cmd_vel nur im RUN-Zustand aus -> dauerhaftes Senden von 0 ist
        # harmlos und liefert beim Loslassen sauber das Stop-Kommando (cmd=0).
        self.cmd_timer = QTimer(self)
        self.cmd_timer.timeout.connect(self.publish_cmd_vel)
        self.cmd_timer.start(33)

        # Nach kurzer Anlaufzeit (DDS/loco_sim/arm_controller verbunden) automatisch
        # die Arme aktivieren und in den BALANCING-Stand gehen. NUR EINMAL feuern (3s):
        # ein zweiter Auto-Start (frueher 6s) loeste einen Ruck/Sturz aus.
        QTimer.singleShot(3000, self._auto_start)

    def init_ui(self):
        main_layout = QVBoxLayout()
        grid = QGridLayout()
        grid.setSpacing(10)
        rows, cols = 5, 5
        self.buttons = {}

        button_actions = {
            (0, 0): ("START", lambda: self.flash_button((0, 0), self.node.pub_start)),

            # BALANCING und WALK liegen nebeneinander und sind GEGENSEITIG EXKLUSIV
            # (Radio): immer genau einer gruen. KEIN automatisches Umschalten mehr —
            # der Nutzer entscheidet. BALANCING = wirklich stationaer (PD, Fuesse
            # geplant, Arme frei bewegbar). WALK = Policy (Joystick faehrt; zentriert
            # steht die Policy am Platz).
            (0, 1): ("START\nBALANCING", lambda: self.radio_loco((0, 1), self.node.pub_start_balancing)),
            (0, 2): ("WALK", lambda: self.radio_loco((0, 2), self.node.pub_start_walking)),

            (1, 1): ("HOMING\nARMS", lambda: self.flash_button((1, 1), self.node.pub_arms_home)),

            (1, 0): ("ENABLE\nMANIPULATION", lambda: self.toggle_button((1, 0), self.node.pub_arms_enabled)),

            # Oben rechts: Marker-Follow (Leader-Follower) an/aus.
            (0, 4): ("MARKER\nFOLLOW", lambda: self.toggle_button((0, 4), self.node.pub_marker_follow)),

            # Stoer-Test (nur Sim): schubst den Roboter in zufaelliger Richtung,
            # um die Stoerunterdrueckung des Balancers zu pruefen.
            (1, 4): ("PUSH\nROBOT", lambda: self.flash_button((1, 4), self.node.pub_push, duration=400)),

            (2, 0): ("OPEN\nLEFT\nHAND", lambda: self.toggle_hand("left", "open", self.node.pub_left_hand)),
            (2, 1): ("CLOSE\nLEFT\nHAND", lambda: self.toggle_hand("left", "close", self.node.pub_left_hand)),
            (2, 2): ("OPEN\nRIGHT\nHAND", lambda: self.toggle_hand("right", "open", self.node.pub_right_hand)),
            (2, 3): ("CLOSE\nRIGHT\nHAND", lambda: self.toggle_hand("right", "close", self.node.pub_right_hand)),
            (2, 4): ("INSPIRE\nFTP\nGUIs", self.open_hand_guis),

            (4, 4): ("EMERGENCY\nSTOP", self.emergency_stop),
        }

        for r in range(rows):
            for c in range(cols):
                btn = QPushButton()
                btn.setMinimumSize(120, 80)

                action = button_actions.get((r, c))
                if action is None:
                    btn.setEnabled(False)
                    btn.setFlat(True)
                    btn.setStyleSheet("""
                        QPushButton {
                            background-color: #1e1e1e;
                            border: 1px solid #333;
                            border-radius: 10px;
                        }
                    """)
                else:
                    label, func = action
                    btn.setText(label)
                    btn.clicked.connect(func)
                    if (r, c) == (4, 4):
                        btn.setStyleSheet("""
                            QPushButton {
                                background-color: #b00000;
                                color: white;
                                font-weight: bold;
                                border: 1px solid #ff4444;
                                border-radius: 10px;
                            }
                            QPushButton:hover {
                                background-color: #ff0000;
                                border: 1px solid #ff6666;
                            }
                        """)

                grid.addWidget(btn, r, c)
                self.buttons[(r, c)] = btn
                self.button_states[(r, c)] = False

        # Loco-Radio-Gruppe: BALANCING (0,1) und WALK (0,2) — immer nur einer gruen.
        self.loco_group = [(0, 1), (0, 2)]

        # Marker-Follow ist per Default aktiv -> Button gruen anzeigen, damit der
        # angezeigte Zustand zum Default des interactive_marker-Node passt.
        self.set_button_active((0, 4), True)

        main_layout.addLayout(grid)

        # ── WALK-Steuerung: Bildschirm-Joystick (vx/vy) + Yaw-Slider ──────────
        self.joystick = VirtualJoystick(180)
        self.yaw_slider = QSlider(Qt.Orientation.Horizontal)
        self.yaw_slider.setMinimum(-100)
        self.yaw_slider.setMaximum(100)
        self.yaw_slider.setValue(0)
        # Beim Loslassen auf 0 zuruecksetzen (Lenkrad-Rueckstellung).
        self.yaw_slider.sliderReleased.connect(lambda: self.yaw_slider.setValue(0))

        walk_box = QVBoxLayout()
        walk_box.addWidget(QLabel("WALK-Joystick  (oben = vorwaerts, links/rechts = seitwaerts) — zentrieren = Stop -> PD-Stand"))
        row = QHBoxLayout()
        row.addStretch(1)
        row.addWidget(self.joystick)
        row.addStretch(1)
        walk_box.addLayout(row)
        walk_box.addWidget(QLabel("Drehen (Yaw)  — springt beim Loslassen auf 0"))
        walk_box.addWidget(self.yaw_slider)
        main_layout.addLayout(walk_box)

        self.setLayout(main_layout)

    def publish_cmd_vel(self):
        """Aktuellen Joystick-/Yaw-Zustand als normierte Velocity senden."""
        msg = Twist()
        msg.linear.x = float(self.joystick.vx)
        msg.linear.y = float(self.joystick.vy)
        msg.angular.z = float(self.yaw_slider.value()) / 100.0
        self.node.pub_cmd_vel.publish(msg)

    def apply_style(self):
        self.setStyleSheet("""
            QPushButton {
                background-color: #2d2d2d;
                color: #ffffff;
                font-size: 15px;
                font-weight: 600;
                border: 1px solid #444;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover:enabled {
                background-color: #3c3c3c;
                border: 1px solid #66b3ff;
            }
            QPushButton:pressed {
                background-color: #1f5fa1;
                border: 1px solid #80c4ff;
            }
            QPushButton:disabled {
                color: #555;
                background-color: #1e1e1e;
                border: 1px solid #2a2a2a;
            }
            QWidget {
                background-color: #111;
            }
            QLabel {
                color: #aaa;
                font-size: 12px;
                font-weight: 400;
            }
        """)

    def set_button_active(self, pos, active=True):
        btn = self.buttons[pos]
        if active:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    border: 1px solid #80ff80;
                    border-radius: 10px;
                }
            """)
        else:
            btn.setStyleSheet("")
            self.apply_style()

        self.button_states[pos] = active

    def flash_button(self, pos, pub, duration=1000):
        """Temporarily activates button for <duration> ms then resets."""
        self.set_button_active(pos, True)
        self.node.publish_bool(pub, True)

        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self.deactivate_button(pos, pub))
        timer.start(duration)

    def deactivate_button(self, pos, pub):
        self.set_button_active(pos, False)
        self.node.publish_bool(pub, False)

    def toggle_button(self, pos, pub):
        new_state = not self.button_states[pos]
        self.set_button_active(pos, new_state)
        self.node.publish_bool(pub, new_state)

    def radio_loco(self, pos, pub):
        """Loco-Radio: genau EINEN der Loco-Buttons (BALANCING/WALK) gruen schalten
        und True publizieren. Kein automatisches Umschalten — nur dieser Klick."""
        for p in self.loco_group:
            self.set_button_active(p, p == pos)
        self.node.publish_bool(pub, True)

    def _auto_start(self):
        """Direkt nach Launch: Arme aktivieren (werden ab jetzt dauerhaft vom
        arm_controller gehalten und sind per rviz bewegbar) und in den BALANCING-
        Stand gehen. So steht der Roboter sofort stabil, ohne Knopfdruck."""
        self.set_button_active((1, 0), True)
        self.node.publish_bool(self.node.pub_arms_enabled, True)
        self.radio_loco((0, 1), self.node.pub_start_balancing)

    # Bind-Mount-Pfad des Repos IM Container (docker-compose: .:/ros2_ws/src/g1pilot).
    # Eine Trigger-Datei hier ist fuer den Host-Watcher (start.sh) sichtbar.
    _HOST_MOUNT = "/ros2_ws/src/g1pilot"
    _GUI_OPEN_TRIGGER = os.path.join(_HOST_MOUNT, ".gui_open_request")

    def open_hand_guis(self):
        """Inspire-FTP-GUIs (Controller + Viewer) im Browser oeffnen.

        ui_interface laeuft IM Container (schlankes ROS-Image, KEIN Browser). Es
        kann deshalb selbst keinen Browser auf dem Host starten. Stattdessen wird
        eine Trigger-Datei im bind-gemounteten Repo angefasst; der Host-Watcher in
        start.sh sieht die neue mtime und oeffnet die GUIs dort, wo ein Browser
        existiert. Laeuft der Node ausnahmsweise direkt auf dem Host (kein Mount),
        wird als Fallback direkt ein Browser gestartet."""
        self.set_button_active((2, 4), True)
        QTimer.singleShot(700, lambda: self.set_button_active((2, 4), False))

        # ── Normalfall: im Container -> Host-Watcher anstossen ───────────────
        if os.path.isdir(self._HOST_MOUNT):
            try:
                with open(self._GUI_OPEN_TRIGGER, "w") as f:
                    f.write(str(time.time()))
                self.node.get_logger().info(
                    "[FTP-GUIs] Oeffnen via Host angefordert (.gui_open_request). "
                    "Der Browser geht auf dem Host auf (start.sh-Watcher).")
            except Exception as e:  # noqa: BLE001
                self.node.get_logger().warn(
                    f"[FTP-GUIs] Trigger-Datei nicht schreibbar ({e}). "
                    "GUIs bitte manuell oeffnen (web/*.html).")
            return

        # ── Fallback: direkt auf dem Host laufend -> Browser selbst starten ──
        # Die Bridge serviert die GUIs per HTTP (:8767). http-URLs mit
        # ?autoconnect=1 funktionieren mit jedem Opener (file:// nicht).
        ctrl = "http://localhost:8767/hand_controller_viewer.html?autoconnect=1"
        view = "http://localhost:8767/inspire_hand_viewer.html?autoconnect=1"

        opener = None
        for cmd in ('xdg-open', 'open', 'sensible-browser', 'x-www-browser',
                    'microsoft-edge', 'microsoft-edge-stable', 'msedge',
                    'firefox', 'google-chrome', 'chromium', 'chromium-browser'):
            if subprocess.run(['which', cmd], capture_output=True).returncode == 0:
                opener = cmd
                break

        if opener is None:
            self.node.get_logger().warn(
                f"[FTP-GUIs] Kein Browser gefunden. Manuell oeffnen:\n  {ctrl}\n  {view}"
            )
            return

        subprocess.Popen([opener, ctrl], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        def _open_viewer():
            time.sleep(1)
            subprocess.Popen([opener, view], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        threading.Thread(target=_open_viewer, daemon=True).start()

    def toggle_hand(self, hand_side, action, pub):
        hand_pair = self.hand_pairs[hand_side]
        this_pos = hand_pair[action]
        other_pos = hand_pair["close" if action == "open" else "open"]

        self.set_button_active(this_pos, True)
        self.set_button_active(other_pos, False)
        self.node.publish_str(pub, action)

    def emergency_stop(self):
        """Turns all buttons OFF and publishes False to all Bool topics."""

        self.node.publish_bool(self.node.pub_start, False)
        self.node.publish_bool(self.node.pub_start_balancing, False)
        self.node.publish_bool(self.node.pub_start_walking, False)
        self.node.publish_bool(self.node.pub_arms_enabled, False)
        self.node.publish_bool(self.node.pub_arms_home, False)
        self.node.publish_bool(self.node.pub_emergency_stop, True)

        for pos in self.buttons:
            if pos != (4, 4):
                self.set_button_active(pos, False)

        btn = self.buttons[(4, 4)]
        btn.setStyleSheet("""
            QPushButton {
                background-color: #ff0000;
                color: white;
                font-weight: bold;
                border: 2px solid #ff6666;
                border-radius: 10px;
            }
        """)


def main():
    rclpy.init()
    node = StreamDeck()

    app = QApplication(sys.argv)
    DarkStyle(app)
    gui = ButtonGUI(node)
    gui.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    app.exec()
    node.destroy_node()
    rclpy.shutdown()
    app.quit()


if __name__ == '__main__':
    main()
