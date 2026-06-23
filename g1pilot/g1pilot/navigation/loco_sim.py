#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loco_sim — Whole-Body-Loco-/Balance-Controller fuer die MuJoCo-Sim.

Ersetzt im Sim das Unitree-Onboard-High-Level (LocoClient.BalanceStand/Move), das
es in MuJoCo nicht gibt. Eine vortrainierte velocity-konditionierte Ganzkoerper-
Policy (unitree_rl_mjlab G1 Velocity, policies/g1_wholebody/policy.onnx) regelt
ALLE 29 Gelenke:

  * cmd = 0   -> der gait_phase-Obs-Term ist 0 (Stand-Signal) -> der Roboter STEHT
                 still und balanciert (headless ~0.01 m Drift / 10 s, driftfrei).
  * cmd != 0  -> die Policy LAEUFT (vor/zurueck/seit/drehen).
  * Walk -> Stand = einfach cmd -> 0 (kein Stepping-Stop, kein modellbasierter PD,
                 kein reserve[]-Schmuggel; headless 7/8 Richtungen sauber).

Eine Policy fuer Stehen UND Laufen -> die fruehere velocity-blinde motion.pt + der
modellbasierte PD-Balancer + Stepping-Stop entfallen. Braucht nur IMU + Gelenk-
Encoder (keine Fusssensoren). Siehe policies/g1_wholebody/ATTRIBUTION.md.

Daten-/Regelfluss (1:1 aus deploy.yaml des Quell-Repos):
  rt/lowstate (IMU + 29 Gelenke) -> Observation(98) -> Policy(ONNX) -> Action(29)
  -> target = default + action*action_scale -> rt/lowcmd (PD je Motor, kp/kd aus
  deploy.yaml). 50 Hz (step_dt 0.02), gait_period 0.6 s, alle Obs-Scales 1.0.

Koerper-Aufteilung (passt zum Bridge-Merge):
  * Beine/Taille 0..14 : Policy-Targets auf rt/lowcmd.
  * Arme 15..28        : Policy-Targets auf rt/lowcmd, ABER vom arm_controller via
    rt/arm_sdk (Weight-Blend) ueberschreibbar. Die Policy sieht die tatsaechlichen
    Armstellungen in ihrer Observation und passt die Beine an -> Grundlage fuer
    Loco-Manipulation (mit Arm-Last/Kiste laufen).

FSM (per Streamdeck-Topics); der Zustand wird auch der Bridge gemeldet (Weld):
  HOLD    : Standby. Bridge haelt die Basis (Weld an), alle Gelenke auf Default-Pose.
  ACTIVE  : Policy aktiv. cmd=0 -> stehen; cmd!=0 -> laufen. Basis frei + aufgestellt.
  DAMP    : Emergency. kp=0, kd=damp -> weich, sanftes Hinsetzen.
  /g1pilot/start_balancing(True) : -> ACTIVE mit cmd=0 (am Platz stehen).
  /g1pilot/start_walking(True)   : -> ACTIVE (Geschwindigkeit via loco_cmd_vel).
  /g1pilot/loco_cmd_vel (Twist)  : normierte [-1,1] Velocity -> phys. Sollwert.
  /g1pilot/emergency_stop(True)  : -> DAMP und Arme aus (/g1pilot/arms/enabled False).
  /g1pilot/start(True)           : -> HOLD (Standby; Bridge haelt Basis wieder).

Aufruf (im g1pilot-Container; im Sim-Bringup automatisch gestartet):
  ros2 run g1pilot loco_sim --ros-args -p interface:=lo
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


# FSM-Zustaende. Der Zustands-Code wird zusaetzlich an die MuJoCo-Bridge gemeldet
# (rt/lowcmd motor_cmd[STATE_IDX].q): 0=HOLD (Basis gehalten), 1=ACTIVE (Basis frei
# + aufgestellt; Policy steht/laeuft), 2=DAMP (Basis frei, kein Reset).
HOLD = "hold"      # Standby: Basis von der Bridge gehalten (Weld an)
ACTIVE = "active"  # Policy aktiv: cmd=0 -> stehen, cmd!=0 -> laufen
DAMP = "damp"      # Emergency: weich/limp (nur kd) -> sanftes Hinsetzen, Basis frei

STATE_IDX = 29     # rt/lowcmd: Zustands-Code an die Bridge (hinter den 29 Gelenken)
LOCO_CODE = {HOLD: 0.0, ACTIVE: 1.0, DAMP: 2.0}

NJ = 29            # G1: 29 Gelenke (Beine 0..11, Taille 12..14, Arme 15..28)


def get_gravity_orientation(quat):
    """Projizierte Gravitation aus dem Pelvis-Quaternion [w,x,y,z] (aufrecht=[0,0,-1]).
    Konvention wie unitree_rl_mjlab/rl_gym; headless gegen die Policy validiert."""
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
        self.declare_parameter("policy", "g1_wholebody")  # Unterordner in policies/
        self.declare_parameter("damp_kd", 8.0)            # kd im DAMP-Zustand
        # Phase-Stand-Schwelle: ||cmd|| (m/s) darunter -> gait_phase=0 (Stand-Signal),
        # exakt wie im Training (mjlab phase(): stand_mask = ||cmd|| < 0.1).
        self.declare_parameter("stand_eps", 0.1)

        interface = self.get_parameter("interface").get_parameter_value().string_value
        policy_name = self.get_parameter("policy").get_parameter_value().string_value
        self.damp_kd = float(self.get_parameter("damp_kd").value)
        self.stand_eps = float(self.get_parameter("stand_eps").value)

        # Lockstep: Regelrate an die Sim-Uhr koppeln (genau decimation Physikschritte
        # pro rt/lowcmd). Dann ist EIN Kommando pro empfangenem rt/lowstate korrekt
        # -> wir takten den Control-Loop auf den lowstate-Eingang.
        self.lockstep = str(os.environ.get("SIM_LOCKSTEP", "")).strip().lower() in (
            "1", "true", "yes", "on")
        self._state_seq = 0

        self._load_policy(policy_name)

        # DDS: im Sim Domain 1 / lo (zentral via utils.common).
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

        # Auf erste LowState warten (sonst kein mode_machine / keine Messung).
        self.get_logger().info("Warte auf rt/lowstate von MuJoCo ...")
        t_wait = time.time()
        while self.low_state is None and rclpy.ok():
            if time.time() - t_wait > 10.0:
                self.get_logger().error(
                    "Keine rt/lowstate empfangen. MuJoCo laeuft? Domain/Interface?")
                break
            time.sleep(0.02)
        if self.low_state is not None:
            self.get_logger().info(f"Verbunden. mode_machine={self.mode_machine}")

        # Laufzeit-Status. Start = HOLD (Bridge haelt die Basis, bevor START kommt).
        self.state = HOLD
        self.action = np.zeros(NJ, dtype=np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)   # phys. Velocity [vx,vy,vyaw] in m/s, rad/s
        self.counter = 0
        self._dbg_grav = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._dbg_gyro = np.zeros(3, dtype=np.float32)

        # Streamdeck-/FSM-Hooks (gleiche Topics wie loco_client auf dem echten Roboter).
        self.create_subscription(Bool, "/g1pilot/start_balancing", self._on_start_balancing, 10)
        self.create_subscription(Bool, "/g1pilot/start_walking", self._on_start_walking, 10)
        self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_emergency, 10)
        self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)
        self.create_subscription(Twist, "/g1pilot/loco_cmd_vel", self._on_cmd_vel, 10)
        # Bei EMERGENCY auch die Arme abschalten (das tat sonst der loco_client).
        self.arms_enabled_pub = self.create_publisher(Bool, "/g1pilot/arms/enabled", 1)

        # PUSH-Stoertest (nur Sim): /g1pilot/push -> UDP-Trigger an den MuJoCo-Prozess.
        self._push_port = int(os.environ.get("SIM_PUSH_PORT", "47900"))
        self._push_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Bool, "/g1pilot/push", self._on_push, 10)

        # Regelschleife in eigenem Thread (feste control_dt, unabhaengig vom ROS-Timer).
        self._run_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._run_thread.start()
        self.get_logger().info(
            f"loco_sim bereit (HOLD = steifer Stand). policy='{policy_name}', "
            f"control_dt={self.control_dt:.3f}s, gait_period={self.gait_period:.2f}s. "
            f"START BALANCING -> ACTIVE/stehen (cmd=0). "
            f"/g1pilot/start_walking + loco_cmd_vel -> laufen.")

    # ── Setup ────────────────────────────────────────────────────────────────
    def _load_policy(self, policy_name):
        import onnxruntime as ort
        import onnx
        share = get_package_share_directory("g1pilot")
        pdir = os.path.join(share, "policies", policy_name)

        # Autoritative Deploy-Config aus dem Quell-Repo (1:1).
        with open(os.path.join(pdir, "deploy.yaml"), "r") as f:
            dep = yaml.safe_load(f)
        self.kps = np.array(dep["stiffness"], dtype=np.float32)
        self.kds = np.array(dep["damping"], dtype=np.float32)
        self.default = np.array(dep["default_joint_pos"], dtype=np.float32)
        self.action_scale = np.array(dep["actions"]["JointPositionAction"]["scale"],
                                     dtype=np.float32)
        self.control_dt = float(dep.get("step_dt", 0.02))
        self.gait_period = float(
            dep["observations"]["gait_phase"]["params"].get("period", 0.6))
        rng = dep["commands"]["base_velocity"]["ranges"]
        self.cmd_x = (float(rng["lin_vel_x"][0]), float(rng["lin_vel_x"][1]))  # (min,max)
        self.cmd_y = (float(rng["lin_vel_y"][0]), float(rng["lin_vel_y"][1]))
        self.cmd_yaw = (float(rng["ang_vel_z"][0]), float(rng["ang_vel_z"][1]))
        assert len(self.kps) == NJ and len(self.default) == NJ, "deploy.yaml: erwarte 29 Gelenke"

        # ONNX-Policy (CPU). Metadaten dienen als Gegenprobe zur deploy.yaml.
        policy_path = os.path.join(pdir, dep.get("policy_file", "policy.onnx"))
        meta = {p.key: p.value for p in onnx.load(policy_path).metadata_props}
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1            # Single-Thread haelt die 50-Hz-Schleife stabil
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(policy_path, sess_options=so,
                                         providers=["CPUExecutionProvider"])
        self.in_name = self.sess.get_inputs()[0].name
        self.num_obs = int(self.sess.get_inputs()[0].shape[-1])
        assert self.num_obs == 98, f"erwarte 98 Obs, ONNX meldet {self.num_obs}"
        self.obs = np.zeros((1, self.num_obs), dtype=np.float32)

        # Warmup (erste Inferenzen JIT-kompilieren intern).
        t0 = time.perf_counter()
        for _ in range(3):
            self.sess.run(None, {self.in_name: self.obs})
        self.get_logger().info(
            f"Policy geladen: {policy_path} (obs={self.num_obs}, actions={NJ}, "
            f"run='{meta.get('run_path','?')}', warmup={1e3*(time.perf_counter()-t0):.0f}ms)")

    # ── DDS / ROS Callbacks ──────────────────────────────────────────────────
    def _on_lowstate(self, msg: LowState_):
        with self._lock:
            self.low_state = msg
            self.mode_machine = int(getattr(msg, "mode_machine", 0))
            self._state_seq += 1

    def _activate(self, zero_cmd, reason):
        if self.low_state is None:
            self.get_logger().warn(f"Kann nicht aktivieren ({reason}): keine rt/lowstate.")
            return
        if zero_cmd:
            with self._lock:
                self.cmd = np.zeros(3, dtype=np.float32)
        self.counter = 0
        self.action[:] = 0.0
        self.state = ACTIVE
        self.get_logger().info(f"{reason} -> ACTIVE (aufstehen + Basis frei).")

    def _on_start_balancing(self, msg: Bool):
        if not msg.data:
            return
        if self.state == ACTIVE:
            with self._lock:
                self.cmd = np.zeros(3, dtype=np.float32)
            self.get_logger().info("START BALANCING: cmd=0 (am Platz stehen).")
            return
        self._activate(zero_cmd=True, reason="START BALANCING (stehen, cmd=0)")

    def _on_start_walking(self, msg: Bool):
        if not msg.data:
            return
        if self.state == ACTIVE:
            self.get_logger().info("Bereits ACTIVE; Geschwindigkeit via loco_cmd_vel.")
            return
        # cmd NICHT nullen -> ein bereits anliegender Joystick-Befehl gilt sofort.
        self._activate(zero_cmd=False, reason="START WALKING")

    def _on_emergency(self, msg: Bool):
        if msg.data:
            self.state = DAMP
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("EMERGENCY STOP -> DAMP + Arme aus.")

    def _on_start(self, msg: Bool):
        if msg.data:
            self.state = HOLD
            self.get_logger().info("START -> Standby (HOLD, steifer Stand).")

    def _on_push(self, msg: Bool):
        if not msg.data:
            return
        try:
            self._push_sock.sendto(b"push", ("127.0.0.1", self._push_port))
            self.get_logger().info("PUSH -> Stoer-Impuls an die Sim (zufaellige Richtung).")
        except OSError as e:
            self.get_logger().warn(f"PUSH konnte nicht gesendet werden: {e}")

    def _on_cmd_vel(self, msg: Twist):
        # Normierte Velocity [-1,1] -> physikalischer Sollwert (m/s, rad/s) gemaess den
        # trainierten Kommandobereichen (asymmetrisch vorwaerts/rueckwaerts). Nur im
        # ACTIVE-Zustand wirksam.
        if self.state != ACTIVE:
            return
        nx = max(-1.0, min(1.0, msg.linear.x))
        ny = max(-1.0, min(1.0, msg.linear.y))
        nz = max(-1.0, min(1.0, msg.angular.z))
        vx = nx * (self.cmd_x[1] if nx >= 0 else -self.cmd_x[0])
        vy = ny * (self.cmd_y[1] if ny >= 0 else -self.cmd_y[0])
        vyaw = nz * (self.cmd_yaw[1] if nz >= 0 else -self.cmd_yaw[0])
        with self._lock:
            self.cmd = np.array([vx, vy, vyaw], dtype=np.float32)

    # ── Regelschleife ────────────────────────────────────────────────────────
    def _control_loop(self):
        diag_n = 0
        diag_busy = 0.0
        diag_worst = 0.0
        diag_grav = np.zeros(3)
        diag_gyro = np.zeros(3)
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
                elif self.state == ACTIVE:
                    self._send_policy()
            except Exception as e:
                self.get_logger().error(f"Regelschleife: {e}")
                self.state = DAMP
            busy = time.perf_counter() - t0

            if self.state == ACTIVE:
                diag_n += 1
                diag_busy += busy
                diag_worst = max(diag_worst, busy)
                diag_grav += self._dbg_grav
                diag_gyro += self._dbg_gyro
                if diag_n >= 100:               # ~2 s bei 50 Hz
                    span = time.perf_counter() - diag_t0
                    hz = diag_n / span if span > 0 else 0.0
                    g = diag_grav / diag_n
                    gy = diag_gyro / diag_n
                    with self._lock:
                        c = self.cmd.copy()
                    standing = float(np.linalg.norm(c)) < self.stand_eps
                    eff = (1.0 / self.control_dt) if self.lockstep else hz
                    self.get_logger().info(
                        f"[timing] wall={hz:.1f}Hz sim-effektiv={eff:.1f}Hz (soll=50) "
                        f"busy_mittel={1e3*diag_busy/diag_n:.1f}ms max={1e3*diag_worst:.1f}ms"
                        + ("  <-- ZU LANGSAM!" if eff < 45 else ""))
                    self.get_logger().info(
                        f"[state] {'STEHEN' if standing else 'LAUFEN'} "
                        f"cmd=[{c[0]:+.2f} {c[1]:+.2f} {c[2]:+.2f}] "
                        f"grav=[{g[0]:+.3f} {g[1]:+.3f} {g[2]:+.3f}] "
                        f"gyro=[{gy[0]:+.3f} {gy[1]:+.3f} {gy[2]:+.3f}]")
                    diag_n = 0
                    diag_busy = 0.0
                    diag_worst = 0.0
                    diag_grav = np.zeros(3)
                    diag_gyro = np.zeros(3)
                    diag_t0 = time.perf_counter()
            else:
                diag_n = 0
                diag_busy = 0.0
                diag_worst = 0.0
                diag_grav = np.zeros(3)
                diag_gyro = np.zeros(3)
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
        # Zustands-Code fuer die Bridge (Managed-Weld): 0=HOLD, 1=ACTIVE, 2=DAMP.
        self.cmd_msg.motor_cmd[STATE_IDX].q = LOCO_CODE.get(self.state, 0.0)
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        self.lowcmd_pub.Write(self.cmd_msg)

    def _send_hold(self):
        # Steifer Stand: alle Gelenke auf Default-Pose (Bridge haelt die Basis via Weld).
        for i in range(NJ):
            mc = self.cmd_msg.motor_cmd[i]
            mc.mode = 1
            mc.q = float(self.default[i])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(self.kps[i])
            mc.kd = float(self.kds[i])
        self._write()

    def _send_damp(self):
        # Alle Motoren weich (kp=0, kd=damp). Roboter sackt langsam, schlaegt nicht.
        for i in range(NJ):
            mc = self.cmd_msg.motor_cmd[i]
            mc.q = 0.0
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = 0.0
            mc.kd = self.damp_kd
        self._write()

    def _build_obs(self):
        ls = self.low_state
        q = np.array([ls.motor_state[i].q for i in range(NJ)], dtype=np.float32)
        dq = np.array([ls.motor_state[i].dq for i in range(NJ)], dtype=np.float32)
        quat = ls.imu_state.quaternion                  # [w,x,y,z], Pelvis
        gyro = np.array(ls.imu_state.gyroscope, dtype=np.float32)   # Body-Frame
        gravity = get_gravity_orientation(quat)
        self._dbg_grav = gravity.copy()
        self._dbg_gyro = gyro.copy()

        with self._lock:
            cmd = self.cmd.copy()
        # gait_phase: bei ||cmd|| < stand_eps auf 0 (Stand-Signal), sonst Phasenuhr.
        if float(np.linalg.norm(cmd)) < self.stand_eps:
            sin_p = cos_p = 0.0
        else:
            phase = (self.counter * self.control_dt % self.gait_period) / self.gait_period
            sin_p = math.sin(2.0 * math.pi * phase)
            cos_p = math.cos(2.0 * math.pi * phase)

        o = self.obs[0]
        o[0:3] = gyro                       # base_ang_vel (scale 1.0)
        o[3:6] = gravity                    # projected_gravity
        o[6:9] = cmd                        # velocity_command (phys.)
        o[9] = sin_p
        o[10] = cos_p
        o[11:11 + NJ] = q - self.default    # joint_pos_rel
        o[11 + NJ:11 + 2 * NJ] = dq         # joint_vel
        o[11 + 2 * NJ:11 + 3 * NJ] = self.action   # last_action
        return self.obs

    def _send_policy(self):
        self.counter += 1
        obs = self._build_obs()
        self.action = self.sess.run(None, {self.in_name: obs})[0][0].astype(np.float32)
        target = self.default + self.action * self.action_scale
        for i in range(NJ):
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
