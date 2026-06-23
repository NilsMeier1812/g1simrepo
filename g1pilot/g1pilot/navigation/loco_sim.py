#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loco_sim — Hybrid-Loco-/Balance-Controller fuer die MuJoCo-Sim.

Ersetzt im Sim das Unitree-Onboard-High-Level (LocoClient.BalanceStand/Move), das
es in MuJoCo nicht gibt. KOMBINIERT zwei Regler, je nach Aufgabe:

  * STAND (am Platz): modellbasierter Knoechel-/Hueft-PD. Haelt die FUESSE GEPLANT
    und richtet die per IMU gemessene Neigung aktiv auf -> Oberkoerper-/Arm-
    Stoerungen werden ohne Schritte abgefangen (man kann die Arme frei bewegen,
    ohne dass er loslaeuft). Braucht nur IMU + Encoder.
  * WALK (laufen): velocity-konditionierte Ganzkoerper-ONNX-Policy
    (policies/g1_wholebody/policy.onnx, unitree_rl_mjlab G1 Velocity, Apache-2.0).
    Laeuft omnidirektional, bremst sauber bis zum Stillstand.

WARUM HYBRID: Die Policy laeuft super und bremst sauber, aber unter DAUERHAFTER
Arm-Bewegung im Stand driftet sie progressiv weg (headless: ~0.15 m nach 6 s,
~0.9 m nach 13 s) — sie 'balanciert' Stoerungen per Schritt. Der PD haelt dagegen
die Fuesse fest. Also: Policy fuers Laufen+Bremsen, PD fuers stationaere Stehen.

Uebergaenge (NUR per Nutzer-Button, KEIN automatisches Umschalten):
  * START BALANCING -> STAND (PD, Fuesse geplant, wirklich stationaer).
  * START WALKING   -> WALK (Policy). Joystick=0 -> die Policy steht am Platz
    (gait_phase=0); zum geplanten Stehen wechselt man bewusst zu BALANCING.
  * Sturz erkannt (IMU-Neigung > Schwelle) -> DAMP (limp), kein Gezappel mehr.

ARME: loco_sim regelt sie NICHT. Sie gehoeren komplett dem arm_controller
(rt/arm_sdk, via rviz/Marker). loco_sim schreibt nur Beine (0..11) + Taille
(12..14) -> kein Arm-"Teleport" bei Zustandswechseln.

FSM (per Streamdeck-Topics); der Zustand wird auch der Bridge gemeldet (Weld):
  HOLD   : Standby. Bridge haelt die Basis (Weld an), Gelenke auf Default-Pose.
  STAND  : PD-Balancer (Basis frei, aufgestellt). Fuesse geplant.
  WALK   : ONNX-Policy (Basis frei). Geschwindigkeit via loco_cmd_vel.
  DAMP   : Emergency / Sturz. kp=0, kd=damp -> weich.
  /g1pilot/start_balancing(True) : -> STAND.
  /g1pilot/start_walking(True)   : -> WALK.
  /g1pilot/loco_cmd_vel (Twist)  : normierte [-1,1] Velocity -> phys. Sollwert.
  /g1pilot/emergency_stop(True)  : -> DAMP + Arme aus.
  /g1pilot/start(True)           : -> HOLD.

Koerper-Aufteilung (passt zum Bridge-Merge): Arme (15..28) werden in BEIDEN Reglern
nur als Fallback auf rt/lowcmd geschrieben und vom arm_controller via rt/arm_sdk
(Weight-Blend) ueberschrieben -> man kann die Arme jederzeit frei fahren.

Aufruf:  ros2 run g1pilot loco_sim --ros-args -p interface:=lo
"""
import math
import os
import socket
import threading
import time

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import init_dds


# FSM-Zustaende. Zustands-Code an die Bridge (rt/lowcmd motor_cmd[STATE_IDX].q):
# 0=HOLD (Basis gehalten), 1=aktiv (Basis frei + aufgestellt; STAND oder WALK),
# 2=DAMP (Basis frei, kein Reset). Der Bridge ist nur wichtig OB balanciert wird.
HOLD = "hold"
STAND = "stand"    # PD-Balancer, Fuesse geplant (stationaer)
WALK = "walk"      # ONNX-Policy (laufen/bremsen)
DAMP = "damp"      # Emergency / Sturz: limp

STATE_IDX = 29
LOCO_CODE = {HOLD: 0.0, STAND: 1.0, WALK: 1.0, DAMP: 2.0}

NJ = 29            # G1: 29 Gelenke

# ── PD-Stand: bewaehrte Bein-Config (1:1 aus dem fruehestabilen g1.yaml) ──
# Die ARME (15..28) regelt loco_sim BEWUSST NICHT — sie gehoeren komplett dem
# arm_controller (rt/arm_sdk, via rviz/Marker). loco_sim schreibt nur Beine (0..11)
# und Taille (12..14). So koennen die Arme nie von loco_sim "teleportiert" werden.
LEG_IDX = list(range(12))
LEG_KP = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], np.float32)
LEG_KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], np.float32)
WAIST_IDX = [12, 13, 14]                           # Taille: yaw, roll, pitch
WAIST_KP = np.array([300, 300, 300], np.float32)
WAIST_KD = np.array([3, 3, 3], np.float32)
WAIST_TARGET = np.zeros(3, np.float32)             # aufrecht
# Indizes im 12er-Bein-Array (hip_pitch,hip_roll,hip_yaw,knee,ankle_pitch,ankle_roll):
L_HIP_PITCH, L_HIP_ROLL, L_HIP_YAW, L_ANKLE_PITCH, L_ANKLE_ROLL = 0, 1, 2, 4, 5
R_HIP_PITCH, R_HIP_ROLL, R_HIP_YAW, R_ANKLE_PITCH, R_ANKLE_ROLL = 6, 7, 8, 10, 11


def get_gravity_orientation(quat):
    """Projizierte Gravitation aus dem Pelvis-Quaternion [w,x,y,z] (aufrecht=[0,0,-1])."""
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    g = np.zeros(3, dtype=np.float32)
    g[0] = 2.0 * (-qz * qx + qw * qy)
    g[1] = -2.0 * (qz * qy + qw * qx)
    g[2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return g


class LocoSim(Node):
    def __init__(self):
        super().__init__("loco_sim")

        self.declare_parameter("interface", "lo")
        self.declare_parameter("policy", "g1_wholebody")
        self.declare_parameter("damp_kd", 8.0)
        # Phase-Stand-Schwelle (||cmd|| m/s) — wie im Training (mjlab phase() < 0.1).
        self.declare_parameter("stand_eps", 0.1)
        # WALK->STAND: nach so vielen Sekunden mit cmd=0 (Policy hat ausgebremst)
        # uebernimmt der PD wieder.
        self.declare_parameter("settle_s", 0.6)
        # Sturz-Erkennung: aufrecht ist proj.grav z=-1; > fall_gz (Neigung ~>60 Grad)
        # ueber fall_debounce_s -> DAMP.
        self.declare_parameter("fall_gz", -0.5)
        self.declare_parameter("fall_debounce_s", 0.3)

        # PD-Balancer-Gains (live tunebar via ros2 param set). Bewaehrte Defaults.
        self.declare_parameter("bal_kp_scale", 10.0)         # Posture-Steifigkeit (Haupthebel)
        self.declare_parameter("bal_ramp_s", 0.4)            # weicher Eintritt Policy->PD
        self.declare_parameter("bal_ankle_kp_pitch", 150.0)
        self.declare_parameter("bal_ankle_kd_pitch", 40.0)
        self.declare_parameter("bal_ankle_kp_roll", 150.0)
        self.declare_parameter("bal_ankle_kd_roll", 40.0)
        self.declare_parameter("bal_ankle_tau_limit", 50.0)
        self.declare_parameter("bal_hip_kp_pitch", 200.0)
        self.declare_parameter("bal_hip_kd_pitch", 40.0)
        self.declare_parameter("bal_hip_kp_roll", 200.0)
        self.declare_parameter("bal_hip_kd_roll", 40.0)
        self.declare_parameter("bal_hip_tau_limit", 80.0)
        self.declare_parameter("bal_yaw_kd", 30.0)

        interface = self.get_parameter("interface").get_parameter_value().string_value
        policy_name = self.get_parameter("policy").get_parameter_value().string_value
        self.damp_kd = float(self.get_parameter("damp_kd").value)
        self.stand_eps = float(self.get_parameter("stand_eps").value)
        self.settle_s = float(self.get_parameter("settle_s").value)
        self.fall_gz = float(self.get_parameter("fall_gz").value)
        self.fall_debounce_s = float(self.get_parameter("fall_debounce_s").value)

        self.lockstep = str(os.environ.get("SIM_LOCKSTEP", "")).strip().lower() in (
            "1", "true", "yes", "on")
        self._state_seq = 0

        self._load_policy(policy_name)

        init_dds(interface, self.get_logger())
        self.low_state = None
        self.mode_machine = 0
        self._lock = threading.Lock()

        self.lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_sub.Init(self._on_lowstate, 10)
        self.lowcmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_pub.Init()

        self.crc = CRC()
        self.cmd_msg = unitree_hg_msg_dds__LowCmd_()
        for i in range(len(self.cmd_msg.motor_cmd)):
            self.cmd_msg.motor_cmd[i].mode = 1

        self.get_logger().info("Warte auf rt/lowstate von MuJoCo ...")
        t_wait = time.time()
        while self.low_state is None and rclpy.ok():
            if time.time() - t_wait > 10.0:
                self.get_logger().error("Keine rt/lowstate. MuJoCo laeuft? Domain/Interface?")
                break
            time.sleep(0.02)
        if self.low_state is not None:
            self.get_logger().info(f"Verbunden. mode_machine={self.mode_machine}")

        # Laufzeit-Status. Start = HOLD.
        self.state = HOLD
        self.action = np.zeros(NJ, dtype=np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)
        self.counter = 0
        self._cmd_zero_since = None        # WALK->STAND-Timer
        self._fall_since = None            # Sturz-Debounce
        self._bal_entering = False         # PD-Eintritts-Rampe
        self._bal_q_start = None
        self._bal_t0 = 0.0
        self._dbg_grav = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._dbg_gyro = np.zeros(3, dtype=np.float32)

        self.create_subscription(Bool, "/g1pilot/start_balancing", self._on_start_balancing, 10)
        self.create_subscription(Bool, "/g1pilot/start_walking", self._on_start_walking, 10)
        self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_emergency, 10)
        self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)
        self.create_subscription(Twist, "/g1pilot/loco_cmd_vel", self._on_cmd_vel, 10)
        self.arms_enabled_pub = self.create_publisher(Bool, "/g1pilot/arms/enabled", 1)

        self._push_port = int(os.environ.get("SIM_PUSH_PORT", "47900"))
        self._push_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Bool, "/g1pilot/push", self._on_push, 10)

        self._run_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._run_thread.start()
        self.get_logger().info(
            f"loco_sim bereit (HOLD). policy='{policy_name}', control_dt={self.control_dt:.3f}s. "
            f"START BALANCING -> STAND (PD, Fuesse geplant). "
            f"START WALKING / loco_cmd_vel -> WALK (Policy).")

    # ── Setup ────────────────────────────────────────────────────────────────
    def _load_policy(self, policy_name):
        import onnxruntime as ort
        import onnx
        share = get_package_share_directory("g1pilot")
        pdir = os.path.join(share, "policies", policy_name)
        with open(os.path.join(pdir, "deploy.yaml"), "r") as f:
            dep = yaml.safe_load(f)
        self.kps = np.array(dep["stiffness"], dtype=np.float32)        # Policy-Gains (WALK)
        self.kds = np.array(dep["damping"], dtype=np.float32)
        self.default = np.array(dep["default_joint_pos"], dtype=np.float32)
        self.action_scale = np.array(dep["actions"]["JointPositionAction"]["scale"],
                                     dtype=np.float32)
        self.control_dt = float(dep.get("step_dt", 0.02))
        self.gait_period = float(dep["observations"]["gait_phase"]["params"].get("period", 0.6))
        rng = dep["commands"]["base_velocity"]["ranges"]
        self.cmd_x = (float(rng["lin_vel_x"][0]), float(rng["lin_vel_x"][1]))
        self.cmd_y = (float(rng["lin_vel_y"][0]), float(rng["lin_vel_y"][1]))
        self.cmd_yaw = (float(rng["ang_vel_z"][0]), float(rng["ang_vel_z"][1]))
        assert len(self.kps) == NJ and len(self.default) == NJ, "deploy.yaml: erwarte 29 Gelenke"

        policy_path = os.path.join(pdir, dep.get("policy_file", "policy.onnx"))
        meta = {p.key: p.value for p in onnx.load(policy_path).metadata_props}
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(policy_path, sess_options=so,
                                         providers=["CPUExecutionProvider"])
        self.in_name = self.sess.get_inputs()[0].name
        self.num_obs = int(self.sess.get_inputs()[0].shape[-1])
        assert self.num_obs == 98, f"erwarte 98 Obs, ONNX meldet {self.num_obs}"
        self.obs = np.zeros((1, self.num_obs), dtype=np.float32)
        for _ in range(3):
            self.sess.run(None, {self.in_name: self.obs})
        self.get_logger().info(
            f"Policy geladen: {policy_path} (run='{meta.get('run_path','?')}')")

    # ── DDS / ROS Callbacks ──────────────────────────────────────────────────
    def _on_lowstate(self, msg: LowState_):
        with self._lock:
            self.low_state = msg
            self.mode_machine = int(getattr(msg, "mode_machine", 0))
            self._state_seq += 1

    def _enter_stand(self, reason):
        if self.low_state is None:
            self.get_logger().warn(f"Kann nicht STAND ({reason}): keine rt/lowstate.")
            return
        with self._lock:
            self.cmd = np.zeros(3, dtype=np.float32)
        self._bal_entering = True           # Rampe: aktuelle Beinpose -> default
        self._bal_q_start = None
        self._cmd_zero_since = None
        self._fall_since = None
        self.state = STAND
        self.get_logger().info(f"{reason} -> STAND (PD, Fuesse geplant).")

    def _enter_walk(self, reason):
        if self.low_state is None:
            self.get_logger().warn(f"Kann nicht WALK ({reason}): keine rt/lowstate.")
            return
        self.counter = 0
        self.action[:] = 0.0
        self._cmd_zero_since = None
        self._fall_since = None
        self.state = WALK
        self.get_logger().info(f"{reason} -> WALK (Policy).")

    def _on_start_balancing(self, msg: Bool):
        if msg.data:
            self._enter_stand("START BALANCING")

    def _on_start_walking(self, msg: Bool):
        if msg.data:
            self._enter_walk("START WALKING")

    def _on_emergency(self, msg: Bool):
        if msg.data:
            self.state = DAMP
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("EMERGENCY STOP -> DAMP + Arme aus.")

    def _on_start(self, msg: Bool):
        if msg.data:
            self.state = HOLD
            self.get_logger().info("START -> Standby (HOLD).")

    def _on_push(self, msg: Bool):
        if not msg.data:
            return
        try:
            self._push_sock.sendto(b"push", ("127.0.0.1", self._push_port))
            self.get_logger().info("PUSH -> Stoer-Impuls an die Sim.")
        except OSError as e:
            self.get_logger().warn(f"PUSH nicht gesendet: {e}")

    def _on_cmd_vel(self, msg: Twist):
        if self.state not in (STAND, WALK):
            return
        nx = max(-1.0, min(1.0, msg.linear.x))
        ny = max(-1.0, min(1.0, msg.linear.y))
        nz = max(-1.0, min(1.0, msg.angular.z))
        vx = nx * (self.cmd_x[1] if nx >= 0 else -self.cmd_x[0])
        vy = ny * (self.cmd_y[1] if ny >= 0 else -self.cmd_y[0])
        vyaw = nz * (self.cmd_yaw[1] if nz >= 0 else -self.cmd_yaw[0])
        with self._lock:
            self.cmd = np.array([vx, vy, vyaw], dtype=np.float32)
        # KEIN Auto-Umschalten: im STAND (PD) ignoriert der Roboter den Joystick und
        # bleibt wirklich stationaer. Laufen startet nur per START WALKING-Button.

    # ── Sturz-Erkennung (IMU-only, gilt fuer STAND und WALK) ─────────────────
    def _fallen(self, gravity):
        if float(gravity[2]) > self.fall_gz:        # zu stark geneigt
            if self._fall_since is None:
                self._fall_since = time.perf_counter()
            elif time.perf_counter() - self._fall_since > self.fall_debounce_s:
                return True
        else:
            self._fall_since = None
        return False

    # ── Regelschleife ────────────────────────────────────────────────────────
    def _control_loop(self):
        diag_n = 0
        diag_t0 = time.perf_counter()
        while rclpy.ok():
            t0 = time.perf_counter()
            try:
                if self.low_state is None:
                    pass
                elif self.state == HOLD:
                    self._send_hold()
                elif self.state == DAMP:
                    self._send_damp()
                elif self.state == STAND:
                    self._send_balance_pd()
                elif self.state == WALK:
                    self._send_policy()
            except Exception as e:
                self.get_logger().error(f"Regelschleife: {e}")
                self.state = DAMP
            busy = time.perf_counter() - t0

            if self.state in (STAND, WALK):
                diag_n += 1
                if diag_n >= 100:
                    span = time.perf_counter() - diag_t0
                    hz = diag_n / span if span > 0 else 0.0
                    eff = (1.0 / self.control_dt) if self.lockstep else hz
                    g = self._dbg_grav
                    self.get_logger().info(
                        f"[{self.state}] eff={eff:.1f}Hz (soll 50) "
                        f"grav=[{g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f}] "
                        f"cmd=[{self.cmd[0]:+.2f} {self.cmd[1]:+.2f} {self.cmd[2]:+.2f}]"
                        + ("  <-- ZU LANGSAM!" if eff < 45 else ""))
                    diag_n = 0
                    diag_t0 = time.perf_counter()
            else:
                diag_n = 0
                diag_t0 = time.perf_counter()

            if self.lockstep:
                target_seq = self._state_seq + 1
                t_wait = time.perf_counter()
                while (self._state_seq < target_seq and rclpy.ok()
                       and (time.perf_counter() - t_wait) < 0.5):
                    time.sleep(0.0002)
            else:
                dt = self.control_dt - busy
                if dt > 0:
                    time.sleep(dt)

    def _write(self):
        self.cmd_msg.mode_pr = 0
        self.cmd_msg.mode_machine = self.mode_machine
        self.cmd_msg.motor_cmd[STATE_IDX].q = LOCO_CODE.get(self.state, 0.0)
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        self.lowcmd_pub.Write(self.cmd_msg)

    def _hold_waist(self, waist_scale=1.0):
        """Taille (12..14) aufrecht halten. waist_scale steift sie zusaetzlich an,
        damit der Oberkoerper nicht nach vorn flopt. Die ARME bleiben unberuehrt
        (arm_controller-Domaene)."""
        for k, idx in enumerate(WAIST_IDX):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(WAIST_TARGET[k])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(WAIST_KP[k]) * waist_scale
            mc.kd = float(WAIST_KD[k]) * math.sqrt(waist_scale)

    def _send_hold(self):
        # Steifer Stand: Beine auf Default (weiche Gains), Taille gehalten. Arme: frei.
        for k, idx in enumerate(LEG_IDX):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(self.default[idx])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(LEG_KP[k])
            mc.kd = float(LEG_KD[k])
        self._hold_waist()
        self._write()

    def _send_damp(self):
        for i in range(len(self.cmd_msg.motor_cmd)):
            mc = self.cmd_msg.motor_cmd[i]
            mc.q = 0.0
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = 0.0
            mc.kd = self.damp_kd
        self._write()

    def _send_balance_pd(self):
        """Modellbasierter Knoechel-/Hueft-Balancer (Fuesse geplant, stationaer).

        Posture-PD auf default_angles (Steifigkeit x bal_kp_scale) + Feedforward-
        DREHMOMENT [Nm] auf Knoechel (primaer) und Huefte (sekundaer), das die per
        IMU gemessene Neigung aktiv aufrichtet. Braucht nur IMU + Encoder ->
        station-keeping per Konstruktion, faengt Oberkoerper-/Arm-Stoerungen ohne
        Schritte. Die Bridge addiert tau zum Posture-PD."""
        ls = self.low_state
        quat = ls.imu_state.quaternion
        gyro = np.array(ls.imu_state.gyroscope, dtype=np.float32)
        g = get_gravity_orientation(quat)
        self._dbg_grav = g.copy()
        self._dbg_gyro = gyro.copy()

        if self._fallen(g):
            self.state = DAMP
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("STURZ erkannt (Stand) -> DAMP + Arme aus.")
            return self._send_damp()

        pitch_err, roll_err = float(g[0]), float(g[1])
        pitch_rate, roll_rate, yaw_rate = float(gyro[1]), float(gyro[0]), float(gyro[2])
        akp_p = self.get_parameter("bal_ankle_kp_pitch").value
        akd_p = self.get_parameter("bal_ankle_kd_pitch").value
        akp_r = self.get_parameter("bal_ankle_kp_roll").value
        akd_r = self.get_parameter("bal_ankle_kd_roll").value
        hkp_p = self.get_parameter("bal_hip_kp_pitch").value
        hkd_p = self.get_parameter("bal_hip_kd_pitch").value
        hkp_r = self.get_parameter("bal_hip_kp_roll").value
        hkd_r = self.get_parameter("bal_hip_kd_roll").value
        a_lim = float(self.get_parameter("bal_ankle_tau_limit").value)
        h_lim = float(self.get_parameter("bal_hip_tau_limit").value)
        yaw_kd = float(self.get_parameter("bal_yaw_kd").value)
        kp_scale = float(self.get_parameter("bal_kp_scale").value)

        # Sanfter Eintritt: aktuelle Beinpose -> default ueber bal_ramp_s blenden,
        # damit der steife PD die Beine nicht aus der (Lauf-)Stellung reisst.
        if self._bal_entering:
            self._bal_q_start = np.array([ls.motor_state[i].q for i in LEG_IDX], dtype=np.float32)
            self._bal_t0 = time.perf_counter()
            self._bal_entering = False
        ramp_s = float(self.get_parameter("bal_ramp_s").value)
        if self._bal_q_start is not None and ramp_s > 1e-3:
            ramp = max(0.0, min(1.0, (time.perf_counter() - self._bal_t0) / ramp_s))
        else:
            ramp = 1.0
        kp_scale_eff = 1.0 + (kp_scale - 1.0) * ramp
        q_start = self._bal_q_start if self._bal_q_start is not None else self.default[:12]

        def clamp(x, lim):
            return max(-lim, min(lim, x))

        # Feedforward [Nm]. Vorzeichen aus der Gelenk-Kinematik (validiert):
        t_ankle_pitch = clamp(akp_p * pitch_err + akd_p * pitch_rate, a_lim)
        t_ankle_roll = clamp(-(akp_r * roll_err + akd_r * roll_rate), a_lim)
        t_hip_pitch = clamp(hkp_p * pitch_err + hkd_p * pitch_rate, h_lim)
        t_hip_roll = clamp(-(hkp_r * roll_err + hkd_r * roll_rate), h_lim)
        t_hip_yaw = clamp(-yaw_kd * yaw_rate, 40.0)

        tau = np.zeros(12, dtype=np.float32)
        tau[L_ANKLE_PITCH] = tau[R_ANKLE_PITCH] = t_ankle_pitch
        tau[L_ANKLE_ROLL] = tau[R_ANKLE_ROLL] = t_ankle_roll
        tau[L_HIP_PITCH] = tau[R_HIP_PITCH] = t_hip_pitch
        tau[L_HIP_ROLL] = tau[R_HIP_ROLL] = t_hip_roll
        tau[L_HIP_YAW] = tau[R_HIP_YAW] = t_hip_yaw
        tau *= ramp
        self.action = np.zeros(NJ, dtype=np.float32)
        self.action[:12] = tau

        kd_scale = math.sqrt(max(kp_scale_eff, 1e-6))
        for k, idx in enumerate(LEG_IDX):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(q_start[k] * (1.0 - ramp) + self.default[idx] * ramp)
            mc.dq = 0.0
            mc.tau = float(tau[k])
            mc.kp = float(LEG_KP[k]) * kp_scale_eff
            mc.kd = float(LEG_KD[k]) * kd_scale
        self._hold_waist(waist_scale=kp_scale_eff)
        self._write()

    def _build_obs(self):
        ls = self.low_state
        q = np.array([ls.motor_state[i].q for i in range(NJ)], dtype=np.float32)
        dq = np.array([ls.motor_state[i].dq for i in range(NJ)], dtype=np.float32)
        quat = ls.imu_state.quaternion
        gyro = np.array(ls.imu_state.gyroscope, dtype=np.float32)
        gravity = get_gravity_orientation(quat)
        self._dbg_grav = gravity.copy()
        self._dbg_gyro = gyro.copy()
        with self._lock:
            cmd = self.cmd.copy()
        if float(np.linalg.norm(cmd)) < self.stand_eps:
            sin_p = cos_p = 0.0
        else:
            phase = (self.counter * self.control_dt % self.gait_period) / self.gait_period
            sin_p = math.sin(2.0 * math.pi * phase)
            cos_p = math.cos(2.0 * math.pi * phase)
        o = self.obs[0]
        o[0:3] = gyro
        o[3:6] = gravity
        o[6:9] = cmd
        o[9] = sin_p
        o[10] = cos_p
        o[11:11 + NJ] = q - self.default
        o[11 + NJ:11 + 2 * NJ] = dq
        o[11 + 2 * NJ:11 + 3 * NJ] = self.action
        return self.obs, gravity, cmd

    def _send_policy(self):
        self.counter += 1
        obs, gravity, cmd = self._build_obs()

        if self._fallen(gravity):
            self.state = DAMP
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("STURZ erkannt (Walk) -> DAMP + Arme aus.")
            return self._send_damp()

        # KEIN Auto-Umschalten mehr: bei cmd=0 STEHT die Policy einfach am Platz
        # (gait_phase=0). Zu wirklich stationaer (Fuesse geplant) wechselt nur der
        # Nutzer per START BALANCING.
        self.action = self.sess.run(None, {self.in_name: obs})[0][0].astype(np.float32)
        target = self.default + self.action * self.action_scale
        # NUR Beine (0..11) + Taille (12..14) aktuieren. Arme (15..28) bleiben dem
        # arm_controller (rt/arm_sdk) ueberlassen -> kein Arm-Teleport beim Laufen.
        for i in range(15):
            mc = self.cmd_msg.motor_cmd[i]
            mc.mode = 1
            mc.q = float(target[i])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(self.kps[i])
            mc.kd = float(self.kds[i])
        self._write()


def main(args=None):
    rclpy.init(args=args)
    node = LocoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
