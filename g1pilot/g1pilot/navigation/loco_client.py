#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

from g1pilot.utils.common import init_dds


def _rpc_get_int(client, api_id):
    try:
        code, data = client._Call(api_id, "{}")
        if code == 0 and data:
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


class G1LocoClient(Node):
    def __init__(self):
        super().__init__("loco_client")
        self.robot_stopped = False
        self.balanced = False
        self.prev_buttons = {}
        self.prev_axis_last = None
        self.control_arms = False

        # Streamdeck-Walk-Zustand (WALK-Button + virtueller Joystick der UI).
        # walk_enabled wirkt als Radio zu START BALANCING: nur wenn True darf
        # der cmd_vel-Pfad Move() rufen. _move_active merkt sich, ob zuletzt
        # ein Move lief, damit StopMove genau EINMAL gesendet wird (kein Spam).
        self.walk_enabled = False
        self._move_active = False
        self._joy_deadman = False        # PS4-Deadman (Button 8) gedrueckt?
        self._cmd_lock = threading.Lock()
        self._cmd_vel = (0.0, 0.0, 0.0)  # normiert [-1,1] aus /g1pilot/loco_cmd_vel
        self._cmd_stamp = None           # time.monotonic() des letzten Empfangs

        self.declare_parameter('use_robot', True)
        self.use_robot = bool(self.get_parameter('use_robot').value)

        self.declare_parameter('interface', 'eth0')
        interface = self.get_parameter('interface').get_parameter_value().string_value
        self.declare_parameter('arm_controlled', 'both')
        self.arm_controlled = self.get_parameter('arm_controlled').get_parameter_value().string_value
        self.declare_parameter('enable_arm_ui', True)
        self.enable_arm_ui = self.get_parameter('enable_arm_ui').get_parameter_value().bool_value

        self.declare_parameter('ik_use_waist', False)
        self.declare_parameter('ik_alpha', 0.2)
        self.declare_parameter('ik_max_dq_step', 0.05)
        self.declare_parameter('arm_velocity_limit', 2.0)

        # Sicherheit/Gehen (Streamdeck- und PS4-Pfad teilen sich die Limits):
        # loco_cmd_vel kommt normiert [-1,1] von der UI -> Skalierung auf m/s bzw.
        # rad/s passiert HIER. cmd_vel_timeout ist der Deadman: bleibt der Stream
        # aus (GUI zu, Verbindung weg), stoppt der Roboter von selbst.
        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vyaw', 0.4)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('cmd_vel_rate_hz', 20.0)
        # Beim Node-Start NICHT in den Roboter-Zustand eingreifen: ein stehender
        # G1 wuerde bei Damp() sofort zusammensacken. Nur fuer Bankett-/Haenge-
        # Tests explizit auf true setzen.
        self.declare_parameter('damp_on_init', False)

        ik_use_waist = self.get_parameter('ik_use_waist').get_parameter_value().bool_value
        ik_alpha = float(self.get_parameter('ik_alpha').value)
        ik_max_dq_step = float(self.get_parameter('ik_max_dq_step').value)
        arm_vel_lim = float(self.get_parameter('arm_velocity_limit').value)

        if self.use_robot:
            init_dds(interface, self.get_logger())
            self.robot = LocoClient()
            self.robot.SetTimeout(10.0)
            self.robot.Init()
            if bool(self.get_parameter('damp_on_init').value):
                self.robot.SetFsmId(4)
                self.robot.Damp()
                self.get_logger().warn("damp_on_init: Roboter wurde in Damp versetzt.")
            self.current_id = self.get_fsm_id()
            self.current_mode = self.get_fsm_mode()
            self.get_logger().info(f"Current FSM ID: {self.current_id}, Mode: {self.current_mode}")
        else:
            self.robot = None
            self.current_id = 4
            self.current_mode = 0
            self.get_logger().info("use_robot:=false -> Not connecting to robot.")

        self.create_subscription(Bool, '/g1pilot/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Bool, '/g1pilot/start', self.start_callback, 10)
        self.create_subscription(Bool, '/g1pilot/start_balancing', self.start_balancing_callback, 10)
        self.create_subscription(Joy, '/g1pilot/joy', self.joystick_callback, 10)
        # Streamdeck-Walk: dieselben Topics, die in der Sim loco_sim bedient.
        self.create_subscription(Bool, '/g1pilot/start_walking', self.start_walking_callback, 10)
        self.create_subscription(Twist, '/g1pilot/loco_cmd_vel', self.cmd_vel_callback, 10)
        rate_hz = max(1.0, float(self.get_parameter('cmd_vel_rate_hz').value))
        self.create_timer(1.0 / rate_hz, self._cmd_vel_tick)

        self.publisher_arms_controlled = self.create_publisher(Bool, '/g1pilot/arms/enabled', 1)
        self.right_gripper_pub = self.create_publisher(String, '/g1pilot/hand_action/right', 1)
        self.left_gripper_pub = self.create_publisher(String, '/g1pilot/hand_action/left', 1)
        self.publisher_homming_arms = self.create_publisher(Bool, '/g1pilot/arms/home', 1)

    def _log_once(self, level, msg, key):
        if getattr(self, key, False):
            return
        setattr(self, key, True)

        logger = self.get_logger()
        if level == "info":
            logger.info(msg)
        elif level == "warn":
            logger.warn(msg)
        elif level == "error":
            logger.error(msg)
        elif level == "debug":
            logger.debug(msg)
        else:
            logger.info(msg)


    def _clear_once(self, key):
        if hasattr(self, key):
            delattr(self, key)

    def _btn_rising(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return msg.buttons[idx] == 1 and prev == 0

    def _btn_falling(self, msg, idx):
        prev = self.prev_buttons.get(idx, 0)
        return msg.buttons[idx] == 0 and prev == 1

    def _axis_edge(self, cur, prev, val):
        return cur == val and prev != val, cur != val and prev == val

    def get_fsm_id(self):
        if not self.use_robot or self.robot is None:
            return 4
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_ID)

    def get_fsm_mode(self):
        if not self.use_robot or self.robot is None:
            return 0
        return _rpc_get_int(self.robot, ROBOT_API_ID_LOCO_GET_FSM_MODE)

    def emergency_callback(self, msg: Bool):
        if msg.data:
            self._log_once("warn", "EMERGENCY STOP ACTIVATED!", "_e_stop_activated_logged")
            self.robot_stopped = True
            self.balanced = False
            self.walk_enabled = False
            self._move_active = False
            if self.use_robot and self.robot is not None:
                self.robot.Damp()
            if self.control_arms:
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
        else:
            self._clear_once("_e_stop_activated_logged")

    def start_callback(self, msg: Bool):
        if self.use_robot and self.robot is not None and msg.data:
            self.walk_enabled = False
            self._stop_move("START/Standby")
            self.robot.SetFsmId(4)
            self._log_once("info", "Switched to FSM ID 4 (Standby)", "_switch_fsm_id_4_logged")
            self.robot_stopped = False
            self.balanced = False

    def start_balancing_callback(self, msg: Bool):
        if not msg.data:
            return
        # Radio-Semantik wie am Streamdeck: BALANCING gewaehlt -> Gehen aus,
        # Roboter bleibt balanciert stehen.
        self.walk_enabled = False
        self._stop_move("START BALANCING")
        if not self.balanced:
            self._log_once("info", "Starting balancing procedure...", "_start_balance_req_logged")
            self.entering_balancing(max_height=0.5, step=0.02)
            self._log_once("info", "Balancing procedure completed.", "_balance_completed_logged")
        else:
            self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_logged")

    # ── Streamdeck-Walk (WALK-Button + virtueller Joystick der UI) ───────────

    def start_walking_callback(self, msg: Bool):
        if not msg.data:
            if self.walk_enabled:
                self.walk_enabled = False
                self._stop_move("WALK deaktiviert")
            return
        if self.robot_stopped or not self.balanced:
            self._log_once("warn",
                           "WALK ignoriert: Roboter ist nicht balanciert "
                           "(erst START -> START BALANCING).",
                           "_walk_not_balanced_logged")
            return
        self._clear_once("_walk_not_balanced_logged")
        if not self.walk_enabled:
            self.walk_enabled = True
            self.get_logger().info(
                "WALK aktiviert: loco_cmd_vel steuert jetzt Move() "
                f"(Limits vx={float(self.get_parameter('max_vx').value):.2f} "
                f"vy={float(self.get_parameter('max_vy').value):.2f} "
                f"vyaw={float(self.get_parameter('max_vyaw').value):.2f}).")

    def cmd_vel_callback(self, msg: Twist):
        # Normiert [-1,1] (Konvention der Streamdeck-UI, identisch zur Sim).
        nx = max(-1.0, min(1.0, float(msg.linear.x)))
        ny = max(-1.0, min(1.0, float(msg.linear.y)))
        nz = max(-1.0, min(1.0, float(msg.angular.z)))
        with self._cmd_lock:
            self._cmd_vel = (nx, ny, nz)
            self._cmd_stamp = time.monotonic()

    def _stop_move(self, reason=""):
        """StopMove genau einmal senden (kein Dauerfeuer an die RPC-Schnittstelle)."""
        if not self._move_active:
            return
        self._move_active = False
        if self.use_robot and self.robot is not None:
            try:
                self.robot.StopMove()
            except Exception as e:
                self.get_logger().error(f"StopMove fehlgeschlagen: {e}")
        if reason:
            self.get_logger().info(f"StopMove ({reason})")

    def _cmd_vel_tick(self):
        """20-Hz-Sicherheitsschleife des Streamdeck-Walk-Pfads.

        Move() laeuft nur solange ALLE Bedingungen gelten (balanciert, nicht
        gestoppt, WALK aktiv, frischer cmd_vel-Stream). Faellt eine weg --
        insbesondere der Deadman-Timeout, wenn die GUI stirbt -- wird einmalig
        StopMove() gesendet. Der PS4-Deadman (Button 8) hat Vorrang, solange
        er gedrueckt ist.
        """
        if self._joy_deadman:
            return
        if not (self.balanced and not self.robot_stopped and self.walk_enabled):
            self._stop_move()
            return
        with self._cmd_lock:
            (nx, ny, nz), stamp = self._cmd_vel, self._cmd_stamp
        timeout = float(self.get_parameter('cmd_vel_timeout').value)
        stale = stamp is None or (time.monotonic() - stamp) > timeout
        if stale or (abs(nx) < 0.03 and abs(ny) < 0.03 and abs(nz) < 0.03):
            self._stop_move("Deadman-Timeout" if stale and self._move_active else "")
            return
        vx = nx * float(self.get_parameter('max_vx').value)
        vy = ny * float(self.get_parameter('max_vy').value)
        vyaw = nz * float(self.get_parameter('max_vyaw').value)
        if self.use_robot and self.robot is not None:
            try:
                self.robot.Move(vx=vx, vy=vy, vyaw=vyaw, continous_move=True)
            except Exception as e:
                self.get_logger().error(f"Move fehlgeschlagen: {e} -> StopMove + Damp")
                try:
                    self.robot.StopMove()
                    self.robot.Damp()
                except Exception:
                    pass
                self.robot_stopped = True
                self.balanced = False
                self.walk_enabled = False
                self._move_active = False
                return
        self._move_active = True

    def joystick_callback(self, msg: Joy):
        try:
            if not self.prev_buttons:
                self.prev_buttons = {i: 0 for i in range(len(msg.buttons))}

            if not self.balanced:
                self._log_once("warn", "Robot is not balanced, cannot move.", "_warn_not_balanced_logged")
            else:
                self._clear_once("_warn_not_balanced_logged")

            if self.robot_stopped:
                self._log_once("warn", "Robot is stopped, cannot move.", "_warn_robot_stopped_logged")
            else:
                self._clear_once("_warn_robot_stopped_logged")

            axis_last = msg.axes[-1] if len(msg.axes) else 0.0
            if self.prev_axis_last is None:
                self.prev_axis_last = axis_last

            up_on, up_off = self._axis_edge(axis_last, self.prev_axis_last, -1.0)
            if up_on:
                if self.use_robot and self.robot is not None:
                    self.robot.SetFsmId(4)
                self._log_once("info", "Switched to FSM ID 4 (Standby)", "_switch_fsm_id_4_logged")
                self.robot_stopped = False
                self.balanced = False
            if up_off:
                self._clear_once("_switch_fsm_id_4_logged")

            if self._btn_rising(msg, 0):
                self.control_arms = not self.control_arms
                if self.control_arms:
                    self._log_once("info", "Enabling arm control mode.", "_enable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=True))
                else:
                    self._log_once("info", "Disabling arm control mode.", "_disable_arm_control_logged")
                    self.publisher_arms_controlled.publish(Bool(data=False))

            if self._btn_rising(msg, 1):
                if self.control_arms:
                    self._log_once("info", "Moving arms to home position.", "_move_arms_home_logged")
                    self.publisher_homming_arms.publish(Bool(data=True))
                else:
                    self._log_once("warn", "Cannot move arms to home, arm control mode is disabled.", "_warn_move_home_no_control_logged")

            if self._btn_rising(msg, 5):
                self._log_once("warn", "Emergency stop button pressed!", "_e_stop_button_pressed_logged")
                self.robot_stopped = True
                self.balanced = False
                if self.use_robot and self.robot is not None:
                    self.robot.Damp()
                self.control_arms = False
                self.publisher_arms_controlled.publish(Bool(data=False))
            if self._btn_falling(msg, 5):
                self._clear_once("_e_stop_button_pressed_logged")

            # Gripper controls
            if msg.axes[4] ==1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open right gripper.", "_open_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="open"))
            if msg.axes[4] ==1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close right gripper.", "_close_right_gripper_logged")
                self.right_gripper_pub.publish(String(data="close"))

            if msg.axes[4] ==-1.0 and self._btn_rising(msg, 3):
                self._log_once("info", "Open left gripper.", "_open_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="open"))
            if msg.axes[4] ==-1.0 and self._btn_falling(msg, 3):
                self._log_once("info", "Close left gripper.", "_close_left_gripper_logged")
                self.left_gripper_pub.publish(String(data="close"))

            if self._btn_rising(msg, 6):
                if not self.balanced:
                    self._log_once("info", "Starting balancing procedure...", "_start_balance_r1_logged")
                    self.entering_balancing(max_height=0.5, step=0.02)
                    self._log_once("info", "Balancing procedure completed.", "_balance_completed_r1_logged")
                else:
                    self._log_once("info", "Already balanced, no action taken.", "_already_balanced_notice_r1_logged")
            if self._btn_falling(msg, 6):
                self._clear_once("_start_balance_r1_logged")
                self._clear_once("_balance_completed_r1_logged")
                self._clear_once("_already_balanced_notice_r1_logged")

            # PS4-Deadman (Button 8): solange gedrueckt steuert der Controller,
            # der Streamdeck-cmd_vel-Pfad pausiert (_cmd_vel_tick prueft das).
            # StopMove nur auf der fallenden Flanke -- frueher wurde es bei
            # losgelassenem Button in JEDEM Joy-Paket gesendet und haette damit
            # auch ein per Streamdeck kommandiertes Gehen dauerhaft abgewuergt.
            self._joy_deadman = (msg.buttons[8] == 1)
            if self._btn_falling(msg, 8) and not self.robot_stopped and self.balanced:
                self._move_active = True   # sicherstellen, dass _stop_move sendet
                self._stop_move("PS4-Deadman losgelassen")

            if msg.buttons[8] == 1 and not self.robot_stopped and self.balanced:
                max_vx = float(self.get_parameter('max_vx').value)
                max_vy = float(self.get_parameter('max_vy').value)
                max_vyaw = float(self.get_parameter('max_vyaw').value)
                vx = round(msg.axes[1] * -max_vx, 2)
                vy = round(msg.axes[0] * -max_vy, 2)
                yaw = round(msg.axes[2] * -max_vyaw, 2)
                self._log_once("info", f"Moving with vx: {vx}, vy: {vy}, yaw: {yaw}", "_moving_logged")
                if self.use_robot and self.robot is not None:
                    if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(yaw) < 0.03:
                        self._stop_move()
                    else:
                        self.robot.Move(vx=vx, vy=vy, vyaw=yaw, continous_move=True)
                        self._move_active = True

            self.prev_buttons = {i: msg.buttons[i] for i in range(len(msg.buttons))}
            self.prev_axis_last = axis_last

        except Exception as e:
            self.get_logger().error(f"Error in joystick_callback: {e}")
            if self.use_robot and self.robot is not None:
                self.robot.StopMove()
                self.robot.Damp()
            self.robot_stopped = True
            self.balanced = False
            self.walk_enabled = False
            self._move_active = False

    def entering_balancing(self, max_height=0.5, step=0.02):
        if not self.use_robot or self.robot is None:
            self.balanced = True
            self.get_logger().info("Sim balancing done (use_robot:=false).")
            return
        height = 0.0
        while height < max_height and not self.robot_stopped:
            height += step
            self.robot.SetStandHeight(height)
            if self.get_fsm_mode() == 0 and height >= 0.2:
                self._log_once("info", f"Reached max height: {height}", "_balance_reach_height_logged")
                break
            elif self.get_fsm_mode() != 0:
                self._log_once("warn", "Problems during balancing, stopping...", "_balance_problem_stop_logged")
                break
        self.robot.BalanceStand(1)
        self.robot.SetStandHeight(height)
        self.robot.Start()
        self.balanced = True


def main(args=None):
    rclpy.init(args=args)
    node = G1LocoClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
