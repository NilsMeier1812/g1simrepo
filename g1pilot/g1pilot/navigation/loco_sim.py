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
  HOLD    : Start-/Standby-Zustand. Die Bridge haelt die Basis (Weld an), Beine
            werden auf Default-Pose gehalten -> Roboter steht sicher, wartet.
  BALANCE : PRIMAER (am Platz stehen + Oberkoerper-Manipulation ausgleichen).
            Default bal_mode=pd: modellbasierter Knoechel-/Hueft-PD haelt eine feste
            Pose -> driftet praktisch nicht (unter Lockstep validiert: ~15 mm/12 s),
            kompensiert Stoerungen ueber IMU-Feedback. Alternativ bal_mode=policy:
            RL-Policy mit cmd=0 (faellt nicht, laeuft aber bei cmd=0 weg -> nur
            bedingt station-keeping). Live umschaltbar via ros2 param set.
  RUN     : Laufen (armiert). COMMAND-GATING: die Lauf-Policy (motion.pt) laeuft NUR
            bei anliegendem Geh-Kommando (loco_cmd_vel); bei zentriertem Joystick
            haelt stattdessen der PD-Stand -> der cmd=0-Eigendrift der Policy wird nie
            sichtbar. Loslassen -> kurz ausrollen -> sanfter Handoff zurueck nach
            BALANCE. So ist der Zyklus Stehen->Gehen->Stehen geschlossen UND driftfrei.
  Deterministik: im Lockstep-Modus (SIM_LOCKSTEP=1) taktet der Control-Loop auf den
  rt/lowstate-Eingang (ein Kommando pro State) -> die Policy sieht garantiert 50 Hz.
  DAMP    : Emergency. Alle Motoren kp=0, kd=damp -> weich, sanftes Hinsetzen.
  BALANCE und RUN melden der Bridge denselben Code (1 = aufstehen + Basis frei);
  der Bridge ist egal, WELCHER Regler balanciert.
  /g1pilot/start_balancing(True) : -> BALANCE (modellbasiert, am Platz stehen).
  /g1pilot/start_walking(True)   : -> RUN (Walking-Policy; Geschwindigkeit via loco_cmd_vel).
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
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import init_dds


# FSM-Zustaende. Der Zustands-Code wird zusaetzlich an die MuJoCo-Bridge gemeldet
# (rt/lowcmd motor_cmd[WEIGHT_IDX].q): 0=HOLD (Basis gehalten), 1=aktiv balancierend
# (Basis frei + aufgestellt), 2=DAMP (Basis frei, kein Reset). Die Bridge interessiert
# nur, OB balanciert wird (Code 1) — nicht WELCHER Regler. Darum teilen sich BALANCE
# und RUN denselben Code 1 (beide: aufstehen + Weld loesen).
HOLD = "hold"      # Standby: Basis von der Bridge gehalten (Weld an)
DAMP = "damp"      # Emergency: weich/limp (nur kd) -> sanftes Hinsetzen, Basis frei
BALANCE = "balance"  # PRIMAER: modellbasierter Knoechel-/Hueft-Balancer (am Platz stehen)
RUN = "run"        # BONUS: Walking-RL-Policy (Laufen via cmd_vel), Basis frei

WEIGHT_IDX = 29    # rt/lowcmd: Zustands-Code an die Bridge (Bein-Kanal, sonst ungenutzt)
LOCO_CODE = {HOLD: 0.0, BALANCE: 1.0, RUN: 1.0, DAMP: 2.0}

# Indizes im 12er-Bein-Array (leg_idx-Reihenfolge, siehe default_angles):
#   0..5 = links  (hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll)
#   6..11 = rechts (dito)
L_HIP_PITCH, L_HIP_ROLL, L_HIP_YAW, L_ANKLE_PITCH, L_ANKLE_ROLL = 0, 1, 2, 4, 5
R_HIP_PITCH, R_HIP_ROLL, R_HIP_YAW, R_ANKLE_PITCH, R_ANKLE_ROLL = 6, 7, 8, 10, 11


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


def yaw_from_quat(quat):
    """Gier-Winkel (um die Welt-z-Achse) aus dem Pelvis-Quaternion [w,x,y,z].
    Damit rotieren wir die Welt-Basis-Geschwindigkeit in den Koerper-Frame
    (vx vorwaerts, vy links) fuer den Stepping-Stop."""
    qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class LocoSim(Node):
    def __init__(self):
        super().__init__("loco_sim")

        self.declare_parameter("interface", "lo")
        self.declare_parameter("policy", "g1")        # Unterordner in policies/
        self.declare_parameter("damp_kd", 8.0)        # kd im DAMP-Zustand
        # BALANCE-Regler: "pd" = modellbasierter Knoechel-/Hueft-PD. Unter Lockstep
        # validiert als BESTER Platzhalter: haelt eine feste Pose (Drift ~15 mm/12 s),
        # driftet per Konstruktion nicht. "policy" = RL-Lauf-Policy mit cmd=0; faellt
        # zwar nicht, LAEUFT aber bei cmd=0 weg (~10 m/12 s -> kein station-keeping).
        # Darum ist "pd" Default fuer am-Platz-Balancieren; die Policy ist fuer RUN
        # (Laufen) da. Live umschaltbar: ros2 param set /loco_sim bal_mode policy.
        self.declare_parameter("bal_mode", "pd")

        # CATCH FALLS: "Stuerze abfangen". Nur im pd-BALANCE relevant. Der PD-Balancer
        # hat einen niedrigen, festen Stoss-Deckel (~50 N vorwaerts), weil er NICHT
        # steppt. Ist catch_falls an (Default), schaltet loco_sim bei drohendem Sturz
        # (Neigung > catch_tilt) automatisch auf die steppfaehige RL-Policy um, die
        # deutlich groessere Stoesse auffaengt (best-effort, keine Garantie). Sobald
        # der Roboter wieder ruhig steht (Neigung < settle_tilt + wenig Drehung fuer
        # settle_s), kehrt er zum praezisen PD-Stand zurueck. Live per Topic
        # /g1pilot/catch_falls oder ros2 param set /loco_sim catch_falls true/false.
        # Schwellen aus dem Headless-Sweep (experiment_push_recovery.py): catch_tilt
        # ~0.15 faengt vorwaerts bis ~120-150 N; zu hoch = zu spaet, zu tief = unnoetig
        # frueh stepppen bei kleinen Stoerungen, die der PD noch ausregelt.
        self.declare_parameter("catch_falls", True)
        self.declare_parameter("catch_tilt", 0.15)          # Neigung -> auf Policy umschalten
        self.declare_parameter("catch_settle_tilt", 0.08)   # Neigung < -> gilt als ruhig
        self.declare_parameter("catch_settle_gyro", 0.6)    # |Gyro| [rad/s] < -> ruhig
        self.declare_parameter("catch_settle_s", 1.5)       # so lange ruhig -> zurueck zu PD
        self._catch_active = False   # True, solange die Policy gerade einen Sturz abfaengt
        self._settle_t0 = None       # Zeitstempel, seit wann es nach dem Fang ruhig ist

        # WALK-STOP-HANDOFF: der entscheidende Gehen->Stehen-Uebergang. Zentriert man
        # im RUN den Joystick (Kommando ~0), bremst die Walking-Policy ab; nach
        # walk_stop_settle_s Ausrollen schaltet loco_sim automatisch zurueck auf den
        # PD-Stand (station-keeping) -> Zyklus Stehen->Gehen->Stehen ist geschlossen.
        # ZEITBASIERT, weil rt/lowstate KEINE Basis-Linear-Geschwindigkeit liefert
        # (nur IMU+Gelenke). Headless validiert (audit_timehandoff.py): settle 1.0-2.0s
        # landen alle "steht am PD" mit Restdrift ~0.2 m/s, den der PD auffaengt; 1.3s
        # ist der beste Kompromiss (niedrigste Restgeschwindigkeit, geringe Restdrift).
        self.declare_parameter("walk_stop_settle_s", 0.5)   # cmd~0 nach Laufen -> RUN->PD
        self.declare_parameter("walk_idle_s", 2.0)          # RUN ohne je Befehl -> PD (Leerlauf-Sicherung)
        self.declare_parameter("walk_stop_eps", 0.06)       # |cmd| darunter = "zentriert"
        self._walk_moved = False     # True, sobald in dieser RUN-Episode wirklich gelaufen
        self._walk_zero_t0 = None    # Zeitstempel, seit wann das Kommando ~0 ist
        # KEIN Command-Gating mehr: im RUN laeuft die rekurrente Geh-Policy (motion.pt)
        # DURCHGEHEND. Headless belegt (diag_gating.py): kontinuierlich uebersteht die
        # Policy aggressives Joystick-Spiel in allen Richtungen ("steht durch"),
        # waehrend per-Schritt-Gating sie zu Fall bringt — jedes Mal, wenn das Kommando
        # unter eps sinkt (Stick durch die Mitte), wurde der LSTM-Zustand genullt und
        # ein steifer PD mitten in den Schritt geworfen -> Sturz. Der cmd=0-Eigendrift
        # (Policy marschiert auf der Stelle, da keine Linear-Velocity in der Obs) wird
        # stattdessen durch den AUTO-HANDOFF aufgefangen: laenger zentrierter Stick ->
        # sanfter Uebergang (Rampe) zum driftfreien PD-Stand.

        # STEPPING-STOP (Gehen->Stehen, der robuste Weg). Ein laufender Biped laesst
        # sich aus BELIEBIGER Richtung nur durch STEPPING anhalten — ein statischer PD
        # kann ankommenden Schwung nicht fangen (headless: seitwaerts/rueckwaerts kommt
        # die Policy bei cmd=0 NIE zur Ruhe; ein zeitbasierter Handoff faengt nur
        # Vorwaerts-Stopps). Darum: bei zentriertem Stick bremst loco_sim AKTIV ueber
        # die Policy ab (Gegen-Kommando ~ -brake_gain * Basis-Geschwindigkeit, aus
        # reserve[]), und uebergibt erst an den PD-Stand, wenn der Roboter langsam genug
        # ist (|v| < brake_vstop) UND im DOPPELSTUETZ steht (beide Fuesse > brake_ds_force).
        # Headless validiert: alle vier Kardinalrichtungen 16/16 (vorher nur vorwaerts).
        # Braucht Fuss-/Geschwindigkeits-Sensorik aus der Bridge (reserve[]); fehlt die,
        # Fallback auf den zeitbasierten Handoff (walk_stop_settle_s).
        self.declare_parameter("brake_gain", 2.5)       # Gegen-Kommando pro m/s (normiert)
        self.declare_parameter("brake_max", 3.0)        # max. Brems-Kommando (normiert)
        self.declare_parameter("brake_vstop", 0.18)     # |v| darunter = langsam genug
        self.declare_parameter("brake_ds_force", 80.0)  # N pro Fuss -> Doppelstuetz
        self.declare_parameter("brake_timeout", 2.5)    # danach lenienter (v-Schwelle +0.12)
        self.declare_parameter("brake_hold", 0.1)       # so lange langsam+Doppelstuetz -> Handoff
        self._braking = False
        self._brake_t0 = 0.0
        self._brake_lowv_t0 = None

        # SANFTER PD-Eintritt: beim Wechsel nach BALANCE (v.a. aus dem Laufen) wuerde
        # der steife Posture-PD die Beine aus der Schrittstellung schlagartig in die
        # Default-Pose reissen -> Lurch ("rennt wild los", headless: v_lurch 0.67 hart
        # vs 0.22 mit Rampe). Darum rampt loco_sim ueber bal_ramp_s die Soll-Pose von
        # der AKTUELLEN Beinstellung -> default_angles, die Steifigkeit 1->bal_kp_scale
        # und das Balance-Tau 0->voll. So uebernimmt der PD ruckfrei.
        self.declare_parameter("bal_ramp_s", 0.4)
        self._bal_entering = False   # True direkt nach einem Wechsel in BALANCE
        self._bal_t0 = 0.0           # Start der Eintritts-Rampe
        self._bal_q_start = None     # Beinpose beim Eintritt (Ausgangspunkt der Rampe)

        interface = self.get_parameter("interface").get_parameter_value().string_value
        policy_name = self.get_parameter("policy").get_parameter_value().string_value
        self.damp_kd = float(self.get_parameter("damp_kd").value)

        # Lockstep: die Sim koppelt die Regelrate an die Sim-Uhr (genau decimation
        # Physikschritte pro rt/lowcmd). Dann ist EIN Kommando pro empfangenem
        # rt/lowstate korrekt (strikte 1:1-Alternation) -> wir takten den
        # Control-Loop auf den lowstate-Eingang statt auf die Wall-Clock. So sieht
        # die 50-Hz-Policy auf jedem PC exakt ihre trainierte Rate.
        self.lockstep = str(os.environ.get("SIM_LOCKSTEP", "")).strip().lower() in (
            "1", "true", "yes", "on")
        self._state_seq = 0

        # ── Modellbasierter Balance-Regler (Knoechel-/Hueft-Strategie) ──────────
        # Feedback auf die per IMU gemessene Neigung (projizierte Gravitation) +
        # Drehrate (Gyro). Stellgroesse ist FEEDFORWARD-DREHMOMENT [Nm] (mc.tau),
        # NICHT ein Positions-Offset: die weichen Policy-Gains (kp_ankle=40) liefern
        # nur ~20 Nm, eine 0.3-rad-Neigung braucht aber ~50 Nm -> zu wenig Autoritaet.
        # Direktes tau (bis ans Modell-Limit ±50 Nm Knoechel) gibt dem Regler die
        # noetige Kraft. Die Bridge addiert tau zum Posture-PD (_pd: tau+kp*..+kd*..).
        #
        # VORZEICHEN aus der Gelenk-Kinematik abgeleitet UND headless in MuJoCo
        # validiert (Achsen y bzw. x):
        #   Pitch: +tau bei Vorwaerts-Neigung (gx>0) -> Koerper kippt zurueck.
        #   Roll : -tau bei Links-Neigung   (gy>0) -> Koerper kippt nach rechts.
        # Falls es in der Sim doch verstaerkt statt abfaengt: kp-Vorzeichen umdrehen.
        #
        # Defaults headless validiert (g1 scene.xml, freie Basis): steht >10 s
        # stabil (Neigung < 0.03), faengt Schuebse bis ~80 N ab, Drehmoment im Limit.
        # ENTSCHEIDEND ist bal_kp_scale: die weichen Lauf-Gains (kp_ankle=40) geben
        # unter Last nach -> der CoM kippt vornueber. Erst ein STEIFER Posture-Hold
        # (kp_scale ~10) haelt die Standpose; das aktive tau-Feedback regelt Stoerungen.
        # Alle Gains live als ROS-Parameter (ros2 param set /loco_sim bal_... X).
        self.declare_parameter("bal_kp_scale", 10.0)         # Posture-Steifigkeit (Beine+Taille) — DER Haupthebel
        self.declare_parameter("bal_ankle_kp_pitch", 150.0)  # Nm pro Einheit Neigung
        self.declare_parameter("bal_ankle_kd_pitch", 40.0)   # Nm pro rad/s
        self.declare_parameter("bal_ankle_kp_roll", 150.0)
        self.declare_parameter("bal_ankle_kd_roll", 40.0)
        self.declare_parameter("bal_hip_kp_pitch", 200.0)    # Huefte-Sekundaerstrategie (validiert: hilft, daempft)
        self.declare_parameter("bal_hip_kd_pitch", 40.0)
        self.declare_parameter("bal_hip_kp_roll", 200.0)
        self.declare_parameter("bal_hip_kd_roll", 40.0)
        self.declare_parameter("bal_ankle_tau_limit", 50.0)  # Knoechel-Motorlimit [Nm]
        self.declare_parameter("bal_hip_tau_limit", 100.0)   # Huefte-Motorlimit [Nm]
        # CoM-Geschwindigkeits-Daempfung (aus reserve[] Basis-v): kippt den Knoechel
        # GEGEN die Rest-Geschwindigkeit beim Fangen -> deutlich robusterer Catch nach
        # dem Stepping-Stop (headless: Teil der 16/16). Plus Yaw-Daempfung ueber hip_yaw
        # gegen die Basis-Drehrate (der reine Pitch/Roll-PD hatte keine Yaw-Autoritaet).
        self.declare_parameter("bal_vel_kv", 120.0)          # Nm pro m/s Basis-Geschwindigkeit
        self.declare_parameter("bal_yaw_kd", 25.0)           # Nm pro rad/s Gier-Rate (hip_yaw)

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
        self._dbg_grav = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self._dbg_gyro = np.zeros(3, dtype=np.float32)
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
        # START BALANCING -> BALANCE (modellbasiert, Hauptfall). start_walking -> RUN
        # (Walking-Policy, Bonus). Beide stellen den Roboter via Bridge auf + loesen den Weld.
        self.create_subscription(Bool, "/g1pilot/start_balancing", self._on_start_balancing, 10)
        self.create_subscription(Bool, "/g1pilot/start_walking", self._on_start_walking, 10)
        self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_emergency, 10)
        self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)
        self.create_subscription(Twist, "/g1pilot/loco_cmd_vel", self._on_cmd_vel, 10)
        # CATCH FALLS an/aus (Streamdeck-Toggle). Setzt den Parameter -> get_parameter
        # bleibt einzige Wahrheit, auch via ros2 param tunebar.
        self.create_subscription(Bool, "/g1pilot/catch_falls", self._on_catch_falls, 10)
        # Bei EMERGENCY auch die Arme abschalten (das tat sonst der loco_client).
        self.arms_enabled_pub = self.create_publisher(Bool, "/g1pilot/arms/enabled", 1)

        # PUSH-Stoertest (nur Sim): /g1pilot/push -> UDP-Trigger an den MuJoCo-Prozess,
        # der dann einen Impuls in zufaelliger Richtung aufbringt. UDP, weil der
        # MuJoCo-Container kein ROS hat; beide laufen host-net -> 127.0.0.1. Port muss
        # zu config.PUSH_UDP_PORT der Sim passen (Default 47900).
        self._push_port = int(os.environ.get("SIM_PUSH_PORT", "47900"))
        self._push_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(Bool, "/g1pilot/push", self._on_push, 10)

        # Regelschleife in eigenem Thread: feste control_dt (50 Hz), unabhaengig von
        # ROS-Timer-Jitter und nah an deploy_real.py.
        self._run_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._run_thread.start()
        self.get_logger().info(
            f"loco_sim bereit (HOLD = steifer Stand). policy='{policy_name}', "
            f"control_dt={self.control_dt:.3f}s. "
            f"START BALANCING -> BALANCE (modellbasiert, am Platz). "
            f"/g1pilot/start_walking -> RUN (Walking-Policy, Bonus).")

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
            self._state_seq += 1   # Lockstep: signalisiert "frischer State da".

    def _on_start_balancing(self, msg: Bool):
        # PRIMAER: RL-Policy als Balance-Regler mit cmd=0 (am Platz stehen +
        # ausgleichen). Das ist exakt der Hardware-Deploy-Pfad (motion.pt deployt
        # via dieselbe DDS-Schnittstelle), robust trainiert. cmd=0 -> station-keeping.
        if not msg.data:
            return
        if self.state == BALANCE:
            self.get_logger().info("Balancieren laeuft bereits.")
            return
        if self.low_state is None:
            self.get_logger().warn("Kann nicht balancieren: keine rt/lowstate.")
            return
        # Code 1 an die Bridge -> Stand-Pose setzen + Weld loesen -> die Policy
        # startet aus einem sauberen, aufrechten Zustand. cmd=0, LSTM/Phase reset.
        with self._lock:
            self.cmd = np.zeros(3, dtype=np.float32)
        self.counter = 0
        self.action[:] = 0.0
        self._reset_policy_state()
        self._catch_active = False
        self._settle_t0 = None
        self._braking = False
        self._brake_lowv_t0 = None
        self._bal_entering = True    # sanfte Eintritts-Rampe scharf schalten
        self.state = BALANCE
        mode = str(self.get_parameter("bal_mode").value).lower()
        how = ("modellbasierter Knoechel-/Hueft-PD, haelt Pose"
               if mode == "pd" else "RL-Policy cmd=0")
        self.get_logger().info(
            f"START BALANCING -> BALANCE ({how}; bal_mode={mode}; aufstehen + Basis frei).")

    def _on_start_walking(self, msg: Bool):
        # BONUS: Walking-RL-Policy. Laufen via /g1pilot/loco_cmd_vel.
        if not msg.data:
            return
        if self.state == RUN:
            self.get_logger().info("Walking laeuft bereits.")
            return
        if self.low_state is None:
            self.get_logger().warn("Kann nicht laufen: keine rt/lowstate.")
            return
        self.counter = 0
        self.action[:] = 0.0
        self._reset_policy_state()
        self._catch_active = False   # bewusstes Laufen -> kein Auto-Zurueck zu PD
        self._settle_t0 = None
        self._walk_moved = False     # noch kein echtes Kommando -> Leerlauf-Sicherung greift
        self._walk_zero_t0 = time.perf_counter()   # ab Eintritt zaehlt der cmd~0-Timer
        self._braking = False
        self._brake_lowv_t0 = None
        self.state = RUN
        self.get_logger().info(
            "START WALKING -> RUN. Policy laeuft durchgehend; Joystick gibt die "
            "Geschwindigkeit. Laenger zentrierter Stick -> sanfter Handoff zum PD-Stand.")

    def _on_catch_falls(self, msg: Bool):
        # Toggle "Stuerze abfangen". Setzt den Parameter (Wahrheit fuer die Schleife).
        self.set_parameters([Parameter("catch_falls", Parameter.Type.BOOL, bool(msg.data))])
        self.get_logger().info(
            f"CATCH FALLS {'AN' if msg.data else 'AUS'} "
            f"({'PD schaltet bei drohendem Sturz auf Policy um' if msg.data else 'reiner PD, kein Auto-Stepping'}).")

    def _on_emergency(self, msg: Bool):
        if msg.data:
            self.state = DAMP
            self._catch_active = False
            self._braking = False
            self._brake_lowv_t0 = None
            self.arms_enabled_pub.publish(Bool(data=False))
            self.get_logger().warn("EMERGENCY STOP -> DAMP + Arme aus.")

    def _on_start(self, msg: Bool):
        if msg.data:
            self.state = HOLD
            self.get_logger().info("START -> Standby (HOLD, steifer Stand).")

    def _on_push(self, msg: Bool):
        # Stoer-Test: ein UDP-Trigger an die Sim -> Impuls in zufaelliger Richtung
        # (Richtung + Kraft macht der MuJoCo-Prozess). Nur ein Datagramm, kein State.
        if not msg.data:
            return
        try:
            self._push_sock.sendto(b"push", ("127.0.0.1", self._push_port))
            self.get_logger().info("PUSH -> Stoer-Impuls an die Sim (zufaellige Richtung).")
        except OSError as e:
            self.get_logger().warn(f"PUSH konnte nicht gesendet werden: {e}")

    def _on_cmd_vel(self, msg: Twist):
        # Normierte Velocity [-1,1]. Nur im RUN-Zustand (Laufen) wirksam; BALANCE
        # haelt cmd=0 (station-keeping) und ignoriert Velocity-Befehle.
        if self.state != RUN:
            return
        vx = max(-1.0, min(1.0, msg.linear.x))
        vy = max(-1.0, min(1.0, msg.linear.y))
        vyaw = max(-1.0, min(1.0, msg.angular.z))
        # Waehrend des Stepping-Stops faehrt _walk_gate_step das Brems-Kommando in
        # self.cmd. Der zentrierte Joystick publiziert aber weiter ~0 -> wuerde das
        # Brems-Kommando ueberschreiben (Race). Darum: beim Bremsen einen zentrierten
        # Stick IGNORIEREN; ein echter neuer Fahrbefehl (>eps) bricht das Bremsen ab.
        if self._braking:
            mag = math.sqrt(vx * vx + vy * vy + vyaw * vyaw)
            if mag <= float(self.get_parameter("walk_stop_eps").value):
                return
            self._braking = False
            self._brake_lowv_t0 = None
        with self._lock:
            self.cmd = np.array([vx, vy, vyaw], dtype=np.float32)

    # ── Stuerze abfangen (PD <-> Policy) ─────────────────────────────────────
    def _catch_step(self):
        """Im pd-BALANCE: schaltet bei drohendem Sturz (Neigung > catch_tilt) auf die
        steppfaehige RL-Policy um (cmd=0, kalt-Reset). Faengt deutlich groessere
        Stoesse als der nicht-steppende PD (best-effort). Nach dem Auffangen, sobald
        es ruhig ist (Neigung+Gyro klein fuer catch_settle_s), zurueck zum PD-Stand.
        Ein bereits laufender Fang wird auch nach Toggle-Aus sauber zu Ende gebracht."""
        enabled = bool(self.get_parameter("catch_falls").value)
        g = self._dbg_grav
        tilt = max(abs(float(g[0])), abs(float(g[1])))

        if (self.state == BALANCE and not self._catch_active and enabled
                and str(self.get_parameter("bal_mode").value).lower() == "pd"):
            if tilt > float(self.get_parameter("catch_tilt").value):
                with self._lock:
                    self.cmd = np.zeros(3, dtype=np.float32)   # cmd=0: nur auffangen, nicht weglaufen
                self._reset_policy_state()
                self.action[:] = 0.0
                self._catch_active = True
                self._settle_t0 = None
                self.state = RUN
                self.get_logger().warn(
                    f"CATCH! Neigung {tilt:.2f} -> Policy faengt den Sturz ab (Stepping, cmd=0).")

        elif self.state == RUN and self._catch_active:
            spin = float(np.linalg.norm(self._dbg_gyro))
            calm = (tilt < float(self.get_parameter("catch_settle_tilt").value)
                    and spin < float(self.get_parameter("catch_settle_gyro").value))
            now = time.perf_counter()
            if calm:
                if self._settle_t0 is None:
                    self._settle_t0 = now
                elif now - self._settle_t0 > float(self.get_parameter("catch_settle_s").value):
                    self._return_to_pd("wieder stabil")
            else:
                self._settle_t0 = None

    def _return_to_pd(self, reason):
        self._reset_policy_state()
        self.action[:] = 0.0
        self._catch_active = False
        self._settle_t0 = None
        self._bal_entering = True
        self.state = BALANCE
        self.get_logger().info(f"CATCH: {reason} -> zurueck zum PD-Stand (station-keeping).")

    # ── Gehen -> Stehen (Stepping-Stop im RUN, KEIN Gating) ──────────────────
    def _read_foot_base(self, ls):
        """Fuss-Normalkraft (links/rechts, N) + Basis-Geschwindigkeit (Welt vx,vy)
        aus reserve[] (von der Bridge). Liefert (fl, fr, vx, vy, valid). valid=False
        wenn reserve leer ist (kein Foot-Sensing -> zeitbasierter Fallback)."""
        try:
            r = ls.reserve
            r0, r1, r2, r3 = int(r[0]), int(r[1]), int(r[2]), int(r[3])
        except Exception:
            return 0.0, 0.0, 0.0, 0.0, False
        if r0 == 0 and r1 == 0 and r2 == 0 and r3 == 0:
            return 0.0, 0.0, 0.0, 0.0, False
        vx = r2 / 1000.0 - 10.0
        vy = r3 / 1000.0 - 10.0
        return float(r0), float(r1), vx, vy, True

    def _handoff_to_balance(self, reason):
        self._reset_policy_state()
        self.action[:] = 0.0
        self._walk_moved = False
        self._walk_zero_t0 = None
        self._braking = False
        self._brake_lowv_t0 = None
        with self._lock:
            self.cmd = np.zeros(3, dtype=np.float32)
        self._bal_entering = True          # ruckfreier PD-Eintritt (Rampe)
        self.state = BALANCE
        self.get_logger().info(
            f"WALK STOP: {reason} -> sanfter Uebergang zum PD-Stand "
            "(station-keeping, Oberkoerper frei fuer Tasks).")

    def _walk_gate_step(self):
        """Im RUN laeuft die Policy DURCHGEHEND (siehe _send_policy-Dispatch). Diese
        Methode macht den Gehen->Stehen-Handoff, OHNE per-Schritt-Gating und OHNE
        LSTM-Reset waehrend des Laufens (das machte die rekurrente Policy instabil).

        Zentriert man den Stick (cmd ~0) nach echtem Laufen, BREMST loco_sim aktiv ueber
        die Policy ab (STEPPING-STOP): Gegen-Kommando ~ -brake_gain * Basis-Geschwindig-
        keit (aus reserve[]), bis |v| < brake_vstop UND Doppelstuetz (beide Fuesse >
        brake_ds_force) -> dann Handoff an den PD-Stand. Das ist der einzige robuste Weg,
        einen laufenden Biped aus beliebiger Richtung anzuhalten (headless: 16/16
        Kardinalrichtungen). Fehlt die Fuss-/v-Sensorik (reserve leer), Fallback auf den
        alten ZEITBASIERTEN Handoff (faengt zuverlaessig nur Vorwaerts-Stopps). Ein
        laufender CATCH-Fang (_catch_active) hat Vorrang."""
        if self.state != RUN or self._catch_active:
            return
        with self._lock:
            cmd_mag = float(np.linalg.norm(self.cmd))
        now = time.perf_counter()
        eps = float(self.get_parameter("walk_stop_eps").value)

        # Aktiv gefahren (und nicht schon am Bremsen) -> weiterlaufen.
        if cmd_mag > eps and not self._braking:
            self._walk_moved = True
            self._walk_zero_t0 = None
            return

        fl, fr, vx_w, vy_w, valid = self._read_foot_base(self.low_state)

        # Kein echtes Laufen ODER keine Sensorik -> ZEITBASIERTER Fallback.
        if not (self._walk_moved and valid):
            if self._walk_zero_t0 is None:
                self._walk_zero_t0 = now
            timeout = (float(self.get_parameter("walk_stop_settle_s").value) if self._walk_moved
                       else float(self.get_parameter("walk_idle_s").value))
            if now - self._walk_zero_t0 > timeout:
                self._handoff_to_balance("Kommando ~0 (zeitbasiert)")
            return

        # STEPPING-STOP: per Stepping abbremsen.
        if not self._braking:
            self._braking = True
            self._brake_t0 = now
            self._brake_lowv_t0 = None
            self.get_logger().info("WALK STOP: bremse per Stepping ab ...")
        yaw = yaw_from_quat(self.low_state.imu_state.quaternion)
        cy, sy = math.cos(yaw), math.sin(yaw)
        vxb = cy * vx_w + sy * vy_w      # vorwaerts
        vyb = -sy * vx_w + cy * vy_w     # links
        vmag = math.hypot(vxb, vyb)
        kb = float(self.get_parameter("brake_gain").value)
        bmax = float(self.get_parameter("brake_max").value)
        bx = max(-bmax, min(bmax, -kb * vxb / float(self.max_cmd[0])))
        by = max(-bmax, min(bmax, -kb * vyb / float(self.max_cmd[1])))
        with self._lock:
            self.cmd = np.array([bx, by, 0.0], dtype=np.float32)   # Policy bremst damit

        vstop = float(self.get_parameter("brake_vstop").value)
        f_th = float(self.get_parameter("brake_ds_force").value)
        elapsed = now - self._brake_t0
        # Nach brake_timeout die v-Schwelle lockern, damit der Stop nicht ewig haengt.
        vth = vstop if elapsed < float(self.get_parameter("brake_timeout").value) else vstop + 0.12
        if vmag < vth and fl > f_th and fr > f_th:
            if self._brake_lowv_t0 is None:
                self._brake_lowv_t0 = now
            elif now - self._brake_lowv_t0 > float(self.get_parameter("brake_hold").value):
                self._handoff_to_balance("abgebremst + Doppelstuetz")
        else:
            self._brake_lowv_t0 = None

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
        diag_grav = np.zeros(3)               # mittlere proj. Gravitation (Lehnung)
        diag_gyro = np.zeros(3)               # mittlerer Gyro (Drehung)
        diag_act = np.zeros(self.num_actions) # mittlere Action (Bias?)
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
                elif self.state == BALANCE:
                    # PRIMAER RL-Policy (cmd=0); "pd" = modellbasierter Fallback.
                    if str(self.get_parameter("bal_mode").value).lower() == "pd":
                        self._send_balance_pd()
                    else:
                        self._send_policy()
                elif self.state == RUN:
                    # KEIN Gating: die rekurrente Geh-Policy laeuft durchgehend (das
                    # stabilste Laufen, alle Richtungen). Der Gehen->Stehen-Handoff
                    # passiert ueber _walk_gate_step (Auto-Wechsel nach BALANCE).
                    self._send_policy()
            except Exception as e:
                self.get_logger().error(f"Regelschleife: {e}")
                self.state = DAMP
            # CATCH FALLS: nach dem Regelschritt (frisch gesetztes _dbg_grav/_dbg_gyro)
            # pruefen, ob bei drohendem Sturz auf die Policy umgeschaltet oder nach dem
            # Auffangen zu PD zurueckgekehrt werden muss.
            if self.low_state is not None:
                self._catch_step()
                self._walk_gate_step()
            busy = time.perf_counter() - t0
            if self.state in (RUN, BALANCE):
                diag_n += 1
                diag_busy_sum += busy
                diag_worst = max(diag_worst, busy)
                diag_obs += self._t_obs
                diag_pol += self._t_pol
                diag_wr += self._t_wr
                diag_grav += self._dbg_grav
                diag_gyro += self._dbg_gyro
                diag_act += np.abs(self.action)
                if busy > self.control_dt:
                    diag_over += 1
                if diag_n >= 100:        # ~2 s bei 50 Hz
                    span = time.perf_counter() - diag_period_last
                    hz = diag_n / span if span > 0 else 0.0
                    # Effektive Sim-Zeit-Rate. Im LOCKSTEP per Konstruktion exakt
                    # 1/control_dt (jede Iteration = decimation Physikschritte =
                    # control_dt Sim-Zeit), unabhaengig von der Wall-Clock-Rate.
                    # Sonst: bei gebremster Sim (factor<1) sieht die Policy pro
                    # Wall-Clock-Schritt mehr Physik -> nur hz/factor muss 50.
                    if self.lockstep:
                        eff = 1.0 / self.control_dt
                        mode_str = "LOCKSTEP"
                    else:
                        eff = hz / self.sim_factor
                        mode_str = f"sim_factor={self.sim_factor:g}"
                    self.get_logger().info(
                        f"[timing] wall={hz:.1f}Hz, sim-effektiv={eff:.1f}Hz (soll=50, "
                        f"{mode_str}), "
                        f"busy_mittel={1e3 * diag_busy_sum / diag_n:.1f}ms "
                        f"max={1e3 * diag_worst:.1f}ms, "
                        f"overruns={diag_over}/{diag_n} | "
                        f"obs={1e3 * diag_obs / diag_n:.2f} "
                        f"policy={1e3 * diag_pol / diag_n:.2f} "
                        f"write={1e3 * diag_wr / diag_n:.2f} ms"
                        + ("  <-- ZU LANGSAM: Sim-effektiv < 45Hz!" if eff < 45 else ""))
                    g = diag_grav / diag_n
                    gy = diag_gyro / diag_n
                    self.get_logger().info(
                        f"[state] grav=[{g[0]:+.3f} {g[1]:+.3f} {g[2]:+.3f}] "
                        f"(aufrecht=[0 0 -1]; gx>0=nach vorn, gy>0=nach links geneigt) "
                        f"gyro=[{gy[0]:+.3f} {gy[1]:+.3f} {gy[2]:+.3f}] "
                        f"|action|_mittel={np.mean(diag_act / diag_n):.3f}")
                    diag_n = 0
                    diag_busy_sum = 0.0
                    diag_over = 0
                    diag_worst = 0.0
                    diag_obs = diag_pol = diag_wr = 0.0
                    diag_grav = np.zeros(3)
                    diag_gyro = np.zeros(3)
                    diag_act = np.zeros(self.num_actions)
                    diag_period_last = time.perf_counter()
            else:
                diag_n = 0
                diag_busy_sum = 0.0
                diag_over = 0
                diag_worst = 0.0
                diag_obs = diag_pol = diag_wr = 0.0
                diag_grav = np.zeros(3)
                diag_gyro = np.zeros(3)
                diag_act = np.zeros(self.num_actions)
                diag_period_last = time.perf_counter()
            if self.lockstep:
                # Lockstep: auf den NAECHSTEN frischen lowstate warten (genau ein
                # Kommando pro State -> 1:1-Alternation mit der Sim). Kein
                # Wall-Clock-Sleep; die Sim-Uhr bestimmt die Rate. Watchdog ~0.5 s,
                # damit der Loop bei Sim-Stopp nicht ewig haengt.
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
        # Zustands-Code fuer die Bridge (Managed-Weld): 0=HOLD, 1=RUN, 2=DAMP.
        self.cmd_msg.motor_cmd[WEIGHT_IDX].q = LOCO_CODE.get(self.state, 0.0)
        self.cmd_msg.crc = self.crc.Crc(self.cmd_msg)
        self.lowcmd_pub.Write(self.cmd_msg)

    def _hold_arm_waist(self, waist_scale=1.0):
        """Taille + Arme auf arm_waist_target halten (Config-Gains).

        waist_scale steift die TAILLE (aw_idx 12,13,14) zusaetzlich an — im Balance-
        Modus noetig, damit der Oberkoerper nicht nach vorn 'flopt' und den CoV
        verschiebt (headless validiert). Arme bleiben nominal (sie werden ohnehin
        vom arm_controller via rt/arm_sdk ueberschrieben)."""
        for k, idx in enumerate(self.aw_idx):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            mc.q = float(self.aw_target[k])
            mc.dq = 0.0
            mc.tau = 0.0
            sc = waist_scale if idx in (12, 13, 14) else 1.0
            mc.kp = float(self.aw_kps[k]) * sc
            mc.kd = float(self.aw_kds[k]) * math.sqrt(sc)

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

    def _send_balance_pd(self):
        """FALLBACK: Modellbasierter Balance-Regler (Knoechel-/Hueft-Strategie).

        Standardmaessig NICHT aktiv (bal_mode=policy). Nur als hardware-ferner
        Notnagel, live aktivierbar via `ros2 param set /loco_sim bal_mode pd`.

        Haelt den Roboter auf der festen Standpose (default_angles) mit einem
        Posture-PD und legt ein FEEDFORWARD-DREHMOMENT [Nm] auf Knoechel (primaer)
        und Huefte (sekundaer), das die per IMU gemessene Neigung aktiv aufrichtet.
        Braucht NUR IMU + Gelenke (genau das, was ueber DDS kommt) -> kein
        Positions-/Velocity-Bedarf, station-keeping per Konstruktion. Reagiert auf
        die gemessene Abweichung und kompensiert damit Oberkoerper-Stoerungen
        automatisch.

        Direktes tau (statt Positions-Offset), weil die weichen Policy-Gains zu wenig
        Knoechel-Autoritaet liefern (~20 Nm; noetig ~50 Nm). Die Bridge addiert tau
        zum Posture-PD (_pd: tau + kp*(q-q_ist) + kd*(dq-dq_ist)).
        """
        ls = self.low_state
        quat = ls.imu_state.quaternion                 # [w,x,y,z], Pelvis
        gyro = np.array(ls.imu_state.gyroscope, dtype=np.float32)
        g = get_gravity_orientation(quat)              # aufrecht=[0,0,-1]
        self._dbg_grav = g.copy()
        self._dbg_gyro = gyro.copy()

        # Neigungs-Fehler: aufrecht -> gx=gy=0. gx>0 = vorn, gy>0 = links geneigt.
        pitch_err = float(g[0])
        roll_err = float(g[1])
        pitch_rate = float(gyro[1])                    # y-Achse = Pitch
        roll_rate = float(gyro[0])                     # x-Achse = Roll

        # Gains live aus den ROS-Parametern (Tuning ohne Rebuild).
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
        kp_scale = float(self.get_parameter("bal_kp_scale").value)

        # Basis-Geschwindigkeit (Koerper-Frame) + Gier-Rate fuer die CoM-/Yaw-Daempfung
        # beim Fangen nach dem Stepping-Stop. Aus reserve[]; fehlt sie -> 0 (reiner
        # Neigungs-PD wie bisher, station-keeping).
        fl_b, fr_b, vx_w, vy_w, vvalid = self._read_foot_base(ls)
        if vvalid:
            yaw_b = yaw_from_quat(quat)
            cyb, syb = math.cos(yaw_b), math.sin(yaw_b)
            vxb = cyb * vx_w + syb * vy_w      # vorwaerts
            vyb = -syb * vx_w + cyb * vy_w     # links
        else:
            vxb = vyb = 0.0
        yaw_rate = float(gyro[2])
        kv = float(self.get_parameter("bal_vel_kv").value)
        yaw_kd = float(self.get_parameter("bal_yaw_kd").value)

        # Sanfter Eintritt: beim ersten Schritt nach dem Wechsel die aktuelle Beinpose
        # als Rampen-Start merken. ramp 0->1 ueber bal_ramp_s blendet Soll-Pose
        # (aktuell->default), Steifigkeit (1->kp_scale) und Balance-Tau (0->voll) ein,
        # damit der steife PD die Beine nicht aus der Schrittstellung reisst.
        if self._bal_entering:
            self._bal_q_start = np.array(
                [ls.motor_state[i].q for i in self.leg_idx], dtype=np.float32)
            self._bal_t0 = time.perf_counter()
            self._bal_entering = False
        ramp_s = float(self.get_parameter("bal_ramp_s").value)
        if self._bal_q_start is not None and ramp_s > 1e-3:
            ramp = (time.perf_counter() - self._bal_t0) / ramp_s
            ramp = max(0.0, min(1.0, ramp))
        else:
            ramp = 1.0
        kp_scale_eff = 1.0 + (kp_scale - 1.0) * ramp     # weich -> steif
        q_start = self._bal_q_start if self._bal_q_start is not None else self.default_angles

        def clamp(x, lim):
            return max(-lim, min(lim, x))

        # Feedforward-Drehmoment [Nm]. Vorzeichen aus der Gelenk-Kinematik:
        #   Knoechel-Pitch: +tau bei Vorwaerts-Neigung (gx>0) -> Koerper kippt zurueck.
        #   Knoechel-Roll : -tau bei Links-Neigung   (gy>0) -> Koerper kippt nach rechts.
        # Roll gleichsinnig auf beide Fuesse (gemeinsames Kippen verschiebt den CoP).
        # + CoM-Geschwindigkeits-Daempfung (kv*v): kippt den Knoechel GEGEN die Rest-
        # geschwindigkeit -> faengt den ankommenden Schwung nach dem Stepping-Stop.
        t_ankle_pitch = clamp(akp_p * pitch_err + akd_p * pitch_rate + kv * vxb, a_lim)
        t_ankle_roll = clamp(-(akp_r * roll_err + akd_r * roll_rate + kv * vyb), a_lim)
        t_hip_pitch = clamp(hkp_p * pitch_err + hkd_p * pitch_rate, h_lim)
        t_hip_roll = clamp(-(hkp_r * roll_err + hkd_r * roll_rate), h_lim)
        t_hip_yaw = clamp(-yaw_kd * yaw_rate, 40.0)    # Yaw-Daempfung ueber hip_yaw

        tau = np.zeros(self.num_actions, dtype=np.float32)
        tau[L_ANKLE_PITCH] = tau[R_ANKLE_PITCH] = t_ankle_pitch
        tau[L_ANKLE_ROLL] = tau[R_ANKLE_ROLL] = t_ankle_roll
        tau[L_HIP_PITCH] = tau[R_HIP_PITCH] = t_hip_pitch
        tau[L_HIP_ROLL] = tau[R_HIP_ROLL] = t_hip_roll
        tau[L_HIP_YAW] = tau[R_HIP_YAW] = t_hip_yaw
        tau *= ramp                                    # Balance-Tau ueber die Rampe einblenden
        self.action = tau                              # fuer die |action|-Diagnose [Nm]

        kd_scale = math.sqrt(max(kp_scale_eff, 1e-6))  # Daempfungsverhaeltnis halten
        for k, idx in enumerate(self.leg_idx):
            mc = self.cmd_msg.motor_cmd[idx]
            mc.mode = 1
            # Soll-Pose von der Eintritts-Beinpose -> default_angles blenden.
            mc.q = float(q_start[k] * (1.0 - ramp) + self.default_angles[k] * ramp)
            mc.dq = 0.0
            mc.tau = float(tau[k])                      # + aktives Balance-Drehmoment
            mc.kp = float(self.kps[k]) * kp_scale_eff
            mc.kd = float(self.kds[k]) * kd_scale
        self._hold_arm_waist(waist_scale=kp_scale_eff)
        self._t_obs = 0.0
        self._t_pol = 0.0
        td = time.perf_counter()
        self._write()
        self._t_wr = time.perf_counter() - td

    def _build_obs(self):
        ls = self.low_state
        qj = np.array([ls.motor_state[i].q for i in self.leg_idx], dtype=np.float32)
        dqj = np.array([ls.motor_state[i].dq for i in self.leg_idx], dtype=np.float32)
        quat = ls.imu_state.quaternion           # [w,x,y,z], Pelvis
        ang_vel = np.array(ls.imu_state.gyroscope, dtype=np.float32)

        gravity = get_gravity_orientation(quat)
        # Roh-Werte fuer die Diagnose merken (vor Skalierung): projizierte Gravitation
        # zeigt eine LEHNUNG (aufrecht = [0,0,-1]; konstantes gx/gy = geneigt), Gyro
        # zeigt Drehung. So unterscheiden wir "lehnt" (Obs-Bias) von "wandert aufrecht".
        self._dbg_grav = gravity.copy()
        self._dbg_gyro = ang_vel.copy()
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
