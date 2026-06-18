#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
loco_sim — Low-Level-Loco-/Balance-Controller fuer die MuJoCo-Sim.

Ersetzt im Sim das Unitree-Onboard-High-Level (LocoClient.BalanceStand), das es
in MuJoCo nicht gibt. Eine vortrainierte unitree_rl_gym-G1-Policy (motion.pt) regelt
die Beine, sodass der Roboter frei steht und balanciert (Velocity-Command = 0).
Spaeter Laufen = nur /g1pilot/loco_cmd_vel != 0 fuettern (keine Architekturaenderung).

Daten-/Regelfluss (identisch zu unitree_rl_gym deploy_real.py):
  rt/lowstate (IMU + Gelenke) -> Observation(47) -> Policy (LSTM) -> Action(12)
  -> target = default_angles + action*action_scale -> rt/lowcmd (PD je Motor).

Koerper-Aufteilung (passt 1:1 zum Bridge-Merge):
  * Beine 0..11   : Policy-Targets (kp/kd aus Config).
  * Taille 12..14 + Arme 15..28 : auf arm_waist_target (0) gehalten — bis der
    arm_controller die Arme via rt/arm_sdk (Weight-Blend) uebernimmt. So sieht die
    Policy exakt die "Arme gehalten"-Dynamik, auf der sie trainiert ist.

FSM (per Streamdeck-Topics); der Zustand wird auch der Bridge gemeldet (Weld):
  HOLD   : Start-/Standby-Zustand. Die Bridge haelt die Basis (Weld an), Beine
           werden auf Default-Pose gehalten -> Roboter steht sicher, wartet.
  RUN    : Policy aktiv. Beim Eintritt stellt die Bridge den Roboter in die
           Stand-Pose und loest den Weld -> freies Stehen/Balancieren.
  DAMP   : Emergency. Alle Motoren kp=0, kd=damp -> weich, sanftes Hinsetzen.
  /g1pilot/start_balancing(True) : -> RUN (Bridge: aufstehen + Basis freigeben).
  /g1pilot/emergency_stop(True)  : -> DAMP und Arme aus (/g1pilot/arms/enabled False).
  /g1pilot/start(True)           : -> HOLD (Standby; Bridge haelt Basis wieder).

Aufruf (im g1pilot-Container; im Sim-Bringup automatisch gestartet):
  ros2 run g1pilot loco_sim --ros-args -p interface:=lo
"""
import math
import os
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
# (rt/lowcmd motor_cmd[WEIGHT_IDX].q): 0=HOLD, 1=RUN, 2=DAMP. Die Bridge haelt im
# off-Modus die Basis (Weld), bis RUN kommt -> dann Stand-Pose + Weld loesen.
HOLD = "hold"      # Standby: Basis von der Bridge gehalten (Weld an)
DAMP = "damp"      # Emergency: weich/limp (nur kd) -> sanftes Hinsetzen, Basis frei
RUN = "run"        # Policy aktiv (Stehen/Balancieren), Basis frei

WEIGHT_IDX = 29    # rt/lowcmd: Zustands-Code an die Bridge (Bein-Kanal, sonst ungenutzt)
LOCO_CODE = {HOLD: 0.0, RUN: 1.0, DAMP: 2.0}


def get_gravity_orientation(quat):
    """Projizierte Gravitation aus dem Pelvis-Quaternion [w,x,y,z].

    Identisch zu unitree_rl_gym common/rotation_helper.get_gravity_orientation
    (G1-IMU sitzt auf dem Pelvis -> keine Frame-Transformation noetig)."""
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
        self.declare_parameter("policy", "g1")        # Unterordner in policies/
        self.declare_parameter("damp_kd", 8.0)        # kd im DAMP-Zustand
        interface = self.get_parameter("interface").get_parameter_value().string_value
        policy_name = self.get_parameter("policy").get_parameter_value().string_value
        self.damp_kd = float(self.get_parameter("damp_kd").value)

        self._load_config_and_policy(policy_name)

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
                self.get_logger().error("Keine rt/lowstate empfangen. MuJoCo laeuft? Domain/Interface?")
                break
            time.sleep(0.02)
        if self.low_state is not None:
            self.get_logger().info(f"Verbunden. mode_machine={self.mode_machine}")

        # Laufzeit-Status. Start = HOLD: steifer Stand auf Default-Pose, damit der
        # Roboter bei freier Basis nicht zusammensackt, bevor START BALANCING kommt.
        self.state = HOLD
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)     # normierte Velocity [vx,vy,vyaw] in [-1,1]
        self.counter = 0
        self._t_obs = self._t_pol = self._t_wr = 0.0  # Rechenzeit-Anteile (Timing-Diagnose)
        # Sim-Realtime-Faktor (nur fuer die Timing-Diagnose): laeuft die MuJoCo-Sim
        # langsamer (z.B. 0.5x), entspricht 1 Policy-Schritt mehr Sim-Physik. Die
        # effektive Sim-Zeit-Rate = wall_hz / factor; nur DARAUF muss 50Hz gelten.
        try:
            self.sim_factor = float(os.environ.get("SIM_REALTIME_FACTOR", "1.0"))
            if self.sim_factor <= 0:
                self.sim_factor = 1.0
        except Exception:
            self.sim_factor = 1.0

        # Streamdeck-/FSM-Hooks (gleiche Topics wie loco_client auf dem echten Roboter).
        self.create_subscription(Bool, "/g1pilot/start_balancing", self._on_start_balancing, 10)
        self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_emergency, 10)
        self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)
        self.create_subscription(Twist, "/g1pilot/loco_cmd_vel", self._on_cmd_vel, 10)
        # Bei EMERGENCY auch die Arme abschalten (das tat sonst der loco_client).
        self.arms_enabled_pub = self.create_publisher(Bool, "/g1pilot/arms/enabled", 1)

        # Regelschleife in eigenem Thread: feste control_dt (50 Hz), unabhaengig von
        # ROS-Timer-Jitter und nah an deploy_real.py.
        self._run_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._run_thread.start()
        self.get_logger().info(
            f"loco_sim bereit (HOLD = steifer Stand). policy='{policy_name}', "
            f"control_dt={self.control_dt:.3f}s. START BALANCING -> Stehen/Balancieren.")

    # ── Setup ────────────────────────────────────────────────────────────────
    def _load_config_and_policy(self, policy_name):
        share = get_package_share_directory("g1pilot")
        pdir = os.path.join(share, "policies", policy_name)
        cfg_path = os.path.join(pdir, "g1.yaml")
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f)

        self.control_dt = float(cfg["control_dt"])
        self.leg_idx = list(cfg["leg_joint2motor_idx"])
        self.kps = np.array(cfg["kps"], dtype=np.float32)
        self.kds = np.array(cfg["kds"], dtype=np.float32)
        self.default_angles = np.array(cfg["default_angles"], dtype=np.float32)
        self.aw_idx = list(cfg["arm_waist_joint2motor_idx"])
        self.aw_kps = np.array(cfg["arm_waist_kps"], dtype=np.float32)
        self.aw_kds = np.array(cfg["arm_waist_kds"], dtype=np.float32)
        self.aw_target = np.array(cfg["arm_waist_target"], dtype=np.float32)
        self.ang_vel_scale = float(cfg["ang_vel_scale"])
        self.dof_pos_scale = float(cfg["dof_pos_scale"])
        self.dof_vel_scale = float(cfg["dof_vel_scale"])
        self.action_scale = float(cfg["action_scale"])
        self.cmd_scale = np.array(cfg["cmd_scale"], dtype=np.float32)
        self.max_cmd = np.array(cfg["max_cmd"], dtype=np.float32)
        self.num_actions = int(cfg["num_actions"])
        self.num_obs = int(cfg["num_obs"])
        self.gait_period = float(cfg.get("gait_period", 0.8))

        # TorchScript-Policy laden (CPU). torch ist im Sim-Image installiert.
        import torch
        self._torch = torch
        # Die Policy-LSTM ist winzig (hidden=64). Auf einem ausgelasteten/langsamen
        # PC ist Multi-Threading hier KONTRAPRODUKTIV: der Thread-Pool-Overhead macht
        # die Inferenz langsamer UND klaut der MuJoCo-Sim Kerne. Single-Thread haelt
        # die 50-Hz-Regelschleife konstanter (siehe Timing-Diagnose im Control-Loop).
        try:
            torch.set_num_threads(1)
        except Exception:
            pass
        # TorchScript-Profiling-Executor ABSCHALTEN. Er instrumentiert jeden
        # forward() und re-optimiert bei dynamischem Control-Flow (LSTM-Loop) staendig
        # nach -> fuer dieses winzige Modell reiner Overhead (gemessen ~29ms/Schritt
        # statt <5ms, deckelt den Loop bei ~30Hz). Der einfache Executor ist hier um
        # ein Vielfaches schneller. (jit.freeze/optimize_for_inference NICHT nutzen:
        # die wuerden den LSTM-hidden_state als Konstante einfrieren -> Reset kaputt.)
        try:
            torch._C._jit_set_profiling_executor(False)
            torch._C._jit_set_profiling_mode(False)
        except Exception:
            pass
        policy_path = os.path.join(pdir, cfg["policy_file"])
        self.policy = torch.jit.load(policy_path, map_location="cpu")
        self.policy.eval()
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        # Diese G1-Policy ist REKURRENT (LSTM): hidden_state/cell_state liegen im
        # Modul und werden bei jedem forward() fortgeschrieben. Sequentielles
        # Aufrufen pro Regelschritt ist daher korrekt (wie deploy_real). Der
        # Zustand MUSS aber zu Beginn jeder Balance-Episode genullt werden, sonst
        # leakt alter LSTM-Zustand ueber DAMP->RUN-Zyklen rein.
        self.recurrent = hasattr(self.policy, "hidden_state") and hasattr(self.policy, "cell_state")
        # Warmup: erste forward()-Aufrufe loesen einmalige TorchScript-Kompilierung
        # aus. Hier abfruehstuecken, damit der erste echte Balance-Schritt nicht
        # 100ms+ haengt. Danach den (durch Warmup veraenderten) LSTM-Zustand nullen.
        t_warm = time.perf_counter()
        with torch.no_grad():
            dummy = torch.zeros(1, self.num_obs, dtype=torch.float32)
            for _ in range(5):
                self.policy(dummy)
        if self.recurrent:
            self.policy.hidden_state.zero_()
            self.policy.cell_state.zero_()
        self.get_logger().info(
            f"Policy geladen: {policy_path} (num_obs={self.num_obs}, "
            f"num_actions={self.num_actions}, recurrent={self.recurrent}, "
            f"warmup={1e3 * (time.perf_counter() - t_warm):.0f}ms)")

    def _reset_policy_state(self):
        """LSTM-Zustand der Policy nullen (Start einer Balance-Episode)."""
        if getattr(self, "recurrent", False):
            with self._torch.no_grad():
                self.policy.hidden_state.zero_()
                self.policy.cell_state.zero_()

    # ── DDS / ROS Callbacks ──────────────────────────────────────────────────
    def _on_lowstate(self, msg: LowState_):
        with self._lock:
            self.low_state = msg
            self.mode_machine = int(getattr(msg, "mode_machine", 0))

    def _on_start_balancing(self, msg: Bool):
        if not msg.data:
            return
        if self.state == RUN:
            self.get_logger().info("Balancieren laeuft bereits.")
            return
        if self.low_state is None:
            self.get_logger().warn("Kann nicht balancieren: keine rt/lowstate.")
            return
        # Direkt in RUN. Die Bridge stellt den Roboter beim RUN-Eintritt in die
        # Stand-Pose und loest den Weld (Code 1) -> die Policy startet aus einem
        # sauberen, aufrechten Zustand.
        self.counter = 0
        self.action[:] = 0.0
        self._reset_policy_state()
        self.state = RUN
        self.get_logger().info("START BALANCING -> RUN (Bridge: aufstehen + Basis freigeben).")

    def _on_emergency(self, msg: Bool):
        if msg.data:
            self.state = DAMP
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("EMERGENCY STOP -> DAMP + Arme aus.")

    def _on_start(self, msg: Bool):
        if msg.data:
            self.state = HOLD
            self.get_logger().info("START -> Standby (HOLD, steifer Stand).")

    def _on_cmd_vel(self, msg: Twist):
        # Normierte Velocity [-1,1]; im ersten Schritt (Stehen) i.d.R. 0.
        vx = max(-1.0, min(1.0, msg.linear.x))
        vy = max(-1.0, min(1.0, msg.linear.y))
        vyaw = max(-1.0, min(1.0, msg.angular.z))
        with self._lock:
            self.cmd = np.array([vx, vy, vyaw], dtype=np.float32)

    # ── Regelschleife ────────────────────────────────────────────────────────
    def _control_loop(self):
        # Timing-Diagnose: misst, ob die Schleife im RUN-Zustand wirklich 50 Hz
        # haelt. Auf einem langsamen PC kann die LSTM-Inferenz + DDS > control_dt
        # dauern -> Loop laeuft zu langsam -> Actions werden zu lange gehalten ->
        # Policy ueberschiesst, driftet, faellt nach Sekunden. Alle ~2 s loggen.
        diag_n = 0
        diag_busy_sum = 0.0          # reine Rechenzeit pro Schritt (ohne sleep)
        diag_over = 0                # Anzahl Schritte mit busy > control_dt (Overrun)
        diag_worst = 0.0
        diag_obs = diag_pol = diag_wr = 0.0   # aufgeschluesselte Rechenzeit
        diag_period_last = time.perf_counter()
        while rclpy.ok():
            t0 = time.perf_counter()
            try:
                if self.low_state is None:
                    pass
                elif self.state == HOLD:
                    self._send_hold()
                elif self.state == DAMP:
                    self._send_damp()
                elif self.state == RUN:
                    self._send_policy()
            except Exception as e:
                self.get_logger().error(f"Regelschleife: {e}")
                self.state = DAMP
            busy = time.perf_counter() - t0
            if self.state == RUN:
                diag_n += 1
                diag_busy_sum += busy
                diag_worst = max(diag_worst, busy)
                diag_obs += self._t_obs
                diag_pol += self._t_pol
                diag_wr += self._t_wr
                if busy > self.control_dt:
                    diag_over += 1
                if diag_n >= 100:        # ~2 s bei 50 Hz
                    span = time.perf_counter() - diag_period_last
                    hz = diag_n / span if span > 0 else 0.0
                    # Effektive Sim-Zeit-Rate: bei gebremster Sim (factor<1) sieht die
                    # Policy pro Wall-Clock-Schritt mehr Physik -> nur hz/factor muss 50.
                    eff = hz / self.sim_factor
                    self.get_logger().info(
                        f"[timing] wall={hz:.1f}Hz, sim-effektiv={eff:.1f}Hz (soll=50, "
                        f"sim_factor={self.sim_factor:g}), "
                        f"busy_mittel={1e3 * diag_busy_sum / diag_n:.1f}ms "
                        f"max={1e3 * diag_worst:.1f}ms, "
                        f"overruns={diag_over}/{diag_n} | "
                        f"obs={1e3 * diag_obs / diag_n:.2f} "
                        f"policy={1e3 * diag_pol / diag_n:.2f} "
                        f"write={1e3 * diag_wr / diag_n:.2f} ms"
                        + ("  <-- ZU LANGSAM: Sim-effektiv < 45Hz!" if eff < 45 else ""))
                    diag_n = 0
                    diag_busy_sum = 0.0
                    diag_over = 0
                    diag_worst = 0.0
                    diag_obs = diag_pol = diag_wr = 0.0
                    diag_period_last = time.perf_counter()
            else:
                diag_n = 0
                diag_busy_sum = 0.0
                diag_over = 0
                diag_worst = 0.0
                diag_obs = diag_pol = diag_wr = 0.0
                diag_period_last = time.perf_counter()
            dt = self.control_dt - busy
            if dt > 0:
                time.sleep(dt)

    def _write(self):
        self.cmd_msg.mode_pr = 0
        self.cmd_msg.mode_machine = self.mode_machine
        # Zustands-Code fuer die Bridge (Managed-Weld): 0=HOLD, 1=RUN, 2=DAMP.
        self.cmd_msg.motor_cmd[WEIGHT_IDX].q = LOCO_CODE.get(self.state, 0.0)
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        self.lowcmd_pub.Write(self.cmd_msg)

    def _hold_arm_waist(self):
        """Taille + Arme auf arm_waist_target halten (Config-Gains)."""
        for k, idx in enumerate(self.aw_idx):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(self.aw_target[k])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(self.aw_kps[k])
            mc.kd = float(self.aw_kds[k])

    def _send_hold(self):
        # Steifer Stand: Beine auf Default-Pose halten (kein aktives Balancieren).
        # Haelt den Roboter aufrecht, bis START BALANCING die Policy aktiviert.
        for k, idx in enumerate(self.leg_idx):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(self.default_angles[k])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(self.kps[k])
            mc.kd = float(self.kds[k])
        self._hold_arm_waist()
        self._write()

    def _send_damp(self):
        # Alle Motoren weich (kp=0, kd=damp). Roboter sackt langsam, schlaegt nicht.
        for i in range(len(self.cmd_msg.motor_cmd)):
            mc = self.cmd_msg.motor_cmd[i]
            mc.q = 0.0
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = 0.0
            mc.kd = self.damp_kd
        self._write()

    def _build_obs(self):
        ls = self.low_state
        qj = np.array([ls.motor_state[i].q for i in self.leg_idx], dtype=np.float32)
        dqj = np.array([ls.motor_state[i].dq for i in self.leg_idx], dtype=np.float32)
        quat = ls.imu_state.quaternion           # [w,x,y,z], Pelvis
        ang_vel = np.array(ls.imu_state.gyroscope, dtype=np.float32)

        gravity = get_gravity_orientation(quat)
        qj_obs = (qj - self.default_angles) * self.dof_pos_scale
        dqj_obs = dqj * self.dof_vel_scale
        ang_vel = ang_vel * self.ang_vel_scale

        count = self.counter * self.control_dt
        phase = (count % self.gait_period) / self.gait_period
        sin_phase = math.sin(2.0 * math.pi * phase)
        cos_phase = math.cos(2.0 * math.pi * phase)

        with self._lock:
            cmd = self.cmd.copy()

        n = self.num_actions
        self.obs[:3] = ang_vel
        self.obs[3:6] = gravity
        self.obs[6:9] = cmd * self.cmd_scale * self.max_cmd
        self.obs[9:9 + n] = qj_obs
        self.obs[9 + n:9 + 2 * n] = dqj_obs
        self.obs[9 + 2 * n:9 + 3 * n] = self.action
        self.obs[9 + 3 * n] = sin_phase
        self.obs[9 + 3 * n + 1] = cos_phase
        return self.obs

    def _send_policy(self):
        self.counter += 1
        ta = time.perf_counter()
        obs = self._build_obs()
        tb = time.perf_counter()
        torch = self._torch
        with torch.no_grad():
            obs_t = torch.from_numpy(obs).unsqueeze(0)
            self.action = self.policy(obs_t).detach().numpy().squeeze()
        tc = time.perf_counter()
        # Reine Rechenzeit-Anteile (ohne Scheduler-Stalls) fuer die Timing-Diagnose:
        # so sehen wir, ob Obs/Policy/Write wirklich teuer sind oder ob der Loop
        # nur durch CPU-Konkurrenz (rviz, sim) verdraengt wird.
        self._t_obs = tb - ta
        self._t_pol = tc - tb
        target = self.default_angles + self.action * self.action_scale

        for k, idx in enumerate(self.leg_idx):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(target[k])
            mc.dq = 0.0
            mc.tau = 0.0
            mc.kp = float(self.kps[k])
            mc.kd = float(self.kds[k])
        self._hold_arm_waist()
        td = time.perf_counter()
        self._write()                       # CRC + DDS-Write
        self._t_wr = time.perf_counter() - td


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
