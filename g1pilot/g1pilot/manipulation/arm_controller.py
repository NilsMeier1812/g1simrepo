#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, math, json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA, String, Float32MultiArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import pinocchio as pin
from pinocchio import SE3



from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)

from g1pilot.utils.ik_solver import G1IKSolver
from g1pilot.navigation import scene_markers as sm
from g1pilot.manipulation.pose_store import PoseStore
from g1pilot.manipulation.arm_planner import plan_arms_joint_path, shortcut_path

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ , LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointIndex,
    DataBuffer,
    init_dds,
)

WORKSPACE = {
    "frame": 'pelvis',
    "left_arm": {
        "left_bottom_front": [0.33, 0.24, 0.02],
        "right_bottom_front": [0.33, 0.07,  0.02],
        "left_bottom_back":   [0.16, 0.24,  0.02],
        "right_bottom_back":  [0.16, 0.07,  0.02],
        "right_top_back":    [0.07, 0.20,  0.20],
        "left_top_back":     [0.07, 0.47,  0.20],
        "right_top_front":  [0.45, 0.11,  0.20],
        "left_top_front":   [0.41, 0.30,  0.20],
    },

    "right_arm": {
        "left_bottom_front": [0.33, -0.24, 0.02],
        "right_bottom_front": [0.33, -0.07,  0.02],
        "left_bottom_back":   [0.16, -0.24,  0.02],
        "right_bottom_back":  [0.16, -0.07,  0.02],
        "right_top_back":    [0.07, -0.20,  0.20],
        "left_top_back":     [0.07, -0.47,  0.20],
        "right_top_front":  [0.45, -0.11,  0.20],
        "left_top_front":   [0.41, -0.30,  0.20],
    },
}


def _yaw_from_R(R: np.ndarray) -> float:
    """Yaw (Z) desde matriz de rotación."""
    return math.atan2(R[1, 0], R[0, 0])


def _mat_to_quat_wxyz(R: np.ndarray):
    q = pin.Quaternion(R)
    return np.array([q.w, q.x, q.y, q.z])


def _quat_wxyz_to_matrix(qwxyz):
    w, x, y, z = qwxyz
    return pin.Quaternion(w, x, y, z).matrix()


class ArmController(Node):
    """
    ROS 2 node controlling G1 arms with external IK (G1IKSolver) and Unitree DDS.

    This node manages the high-level control of both G1-29 arms by combining
    inverse kinematics (Pinocchio-based), filtered goal tracking, end-effector
    auto-calibration, and direct low-level command publishing via Unitree's DDS
    LowCmd interface. It also supports simulation mode through /joint_states
    publishing when `use_robot=False`.

    Parameters
    ----------
    use_robot : bool
        Enables physical robot control (DDS) if True; simulation otherwise.
    interface : str
        Ethernet interface for DDS communication.
    arm_velocity_limit : float
        Maximum allowed joint velocity.
    rate_hz : float
        Main loop update frequency.
    ik_world_frame : str
        Reference frame for IK computation.
    ik_alpha : float
        Exponential smoothing coefficient for joint updates.
    ik_goal_filter_alpha : float
        Low-pass filter coefficient for goal smoothing.
    ik_orientation_mode : str
        Orientation mode for IK ('full', 'yaw-only', etc.).
    ik_max_ori_step_rad : float
        Maximum allowed orientation step in radians per iteration.
    ee_auto_calibrate : bool
        Enables end-effector offset auto-calibration.
    auto_reissue_goals : bool
        Automatically reapply last goals after homing.
    goal_pos_tol : float
        Tolerance (m) for position convergence check.
    goal_ori_tol_deg : float
        Tolerance (deg) for orientation convergence check.
    ee_offset_right_xyz : list[float]
        Static XYZ offset for right-hand calibration.
    ee_offset_right_rpy_deg : list[float]
        Static RPY offset (deg) for right-hand calibration.
    ee_offset_left_xyz : list[float]
        Static XYZ offset for left-hand calibration.
    ee_offset_left_rpy_deg : list[float]
        Static RPY offset (deg) for left-hand calibration.
    """



    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("Arm Controller Node started.")

        self.declare_parameter("use_robot", True)
        self.declare_parameter("interface", "eth0")
        # Gelenk-Geschwindigkeitslimit [rad/s]. Frueher 5.0 -- viel zu schnell
        # fuer echte Hardware (und der Launcher hat seinen 2.0-Default nie an
        # den Node uebergeben, d.h. es galt immer 5.0).
        self.declare_parameter("arm_velocity_limit", 1.5)
        # Kartesisches Speedlimit [m/s] fuer Hand-TCP UND Ellbogen. 0.25 m/s =
        # "reduced speed" nach ISO 10218-1 / ISO TS 15066 (Industriewert fuer
        # Betrieb mit Personen im Schutzraum). Faengt den Fall ab, dass der
        # IK-Solver eine wirre Konfiguration vorschlaegt: der Arm darf da
        # hinfahren, aber nur mit dieser Geschwindigkeit. 0.0 = aus.
        self.declare_parameter("ee_velocity_limit", 0.25)
        # Selbstkollisions-Gate: kommandierte Konfiguration wird VOR dem Senden
        # gegen Selbstkollision geprueft (Marge s. ik_solver.collision_margin);
        # bei Verletzung haelt der Arm an, statt in den Koerper zu fahren.
        self.declare_parameter("self_collision_gate", True)
        # Umgebungs-Kollisions-Gate: dieselbe Pruefung, aber gegen die Objekte
        # aus /scene_markers (Hindernisse -> ausweichen, Greif-Objekte -> Hand
        # darf ran, siehe ik_solver.environment_command_in_collision). Eigener
        # Schalter, unabhaengig von self_collision_gate (siehe g1pilot/
        # SCENE_BRIDGE.md Abschnitt 6).
        self.declare_parameter("environment_collision_gate", True)
        # Toleranz [rad], ab der ein Wegpunkt der GEPLANTEN Bewegung
        # (Positionsspeicher, siehe SCENE_BRIDGE.md Abschnitt 9) als erreicht
        # gilt und der naechste Wegpunkt drankommt.
        self.declare_parameter("planned_motion_tolerance", 0.02)
        # Daempfung [kd] der Arm-Gelenke im E-Stop/Slack-Zustand. kp=0/tau=0 ->
        # keine Positionshaltung; kleines kd -> die Arme sacken GEDAEMPFT statt
        # frei zu fallen (an Seilen abgefangen). 0.0 = voellig frei (harter Fall).
        self.declare_parameter("estop_arm_kd", 3.0)
        self.declare_parameter("rate_hz", 250.0)
        self.declare_parameter("ik_world_frame", "pelvis")
        self.declare_parameter("ik_alpha", 0.2)
        self.declare_parameter("ik_goal_filter_alpha", 0.25)
        self.declare_parameter("ik_orientation_mode", "full")
        self.declare_parameter("ik_max_ori_step_rad", 0.35)
        # Staerke der IK-Nullraum-Regularisierung zur Ruhe-Pose (0 = aus). Verhindert,
        # dass der redundante Arm beim Marker-Ziehen in den zum Koerper gefalteten
        # Zweig faellt. Live tunebar.
        self.declare_parameter("ik_null_space_gain", 0.15)
        # Schwerkraft-Feedforward [Nm] auf die Arm-Gelenke (aus dem Pinocchio-
        # Modell). Ohne haengt der Arm PD-bedingt ~0.05 rad unter dem Sollwert
        # (mit den schwereren Inspire-FTP-Haenden ~2-4 cm an der Hand) -> die
        # Hand "haelt nicht stabil an der Stelle". Live abschaltbar.
        self.declare_parameter("gravity_comp", True)
        self.declare_parameter("ee_auto_calibrate", True)
        # arm_sdk-Gewichts-Rampe (motor_cmd[29].q): 0=Roboter-eigene Steuerung,
        # 1=arm_sdk uebernimmt. Auf dem echten G1 muss das Gewicht beim Enable
        # sanft 0->1 und beim Disable 1->0 gefahren werden, sonst reisst die
        # Uebergabe an den Armen (Unitree-Konvention, s. g1_arm7_sdk-Beispiel).
        # 0.0 = sofort umschalten (Sim-Verhalten, wird vom Sim-Bringup gesetzt).
        self.declare_parameter("arm_weight_ramp_up_s", 2.0)
        self.declare_parameter("arm_weight_ramp_down_s", 2.0)


        self.declare_parameter("ee_offset_right_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_right_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_rpy_deg", [0.0, 0.0, 0.0])

        self.use_robot = bool(self.get_parameter("use_robot").value)
        self.interface = str(self.get_parameter("interface").value)
        self.arm_velocity_limit = float(self.get_parameter("arm_velocity_limit").value)
        self.ee_velocity_limit = float(self.get_parameter("ee_velocity_limit").value)
        self.self_collision_gate = bool(self.get_parameter("self_collision_gate").value)
        self.environment_collision_gate = bool(self.get_parameter("environment_collision_gate").value)
        self.planned_motion_tolerance = float(self.get_parameter("planned_motion_tolerance").value)
        self.estop_arm_kd = float(self.get_parameter("estop_arm_kd").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.frame = str(self.get_parameter("ik_world_frame").value)
        self.ik_alpha = float(self.get_parameter("ik_alpha").value)
        self.ik_goal_filter_alpha = float(self.get_parameter("ik_goal_filter_alpha").value)
        self.ik_orientation_mode = str(self.get_parameter("ik_orientation_mode").value).lower()
        self.ik_max_ori_step_rad = float(self.get_parameter("ik_max_ori_step_rad").value)
        self.ee_auto_calibrate = bool(self.get_parameter("ee_auto_calibrate").value)

        self.declare_parameter("auto_reissue_goals", True)
        self.declare_parameter("goal_pos_tol", 0.01)
        self.declare_parameter("goal_ori_tol_deg", 3.0)

        self.auto_reissue_goals = bool(self.get_parameter("auto_reissue_goals").value)
        self.goal_pos_tol = float(self.get_parameter("goal_pos_tol").value)
        self.goal_ori_tol_deg = float(self.get_parameter("goal_ori_tol_deg").value)

        def _pvec(name):
            v = self.get_parameter(name).value
            return np.array(v, dtype=float)

        self._ee_off_right_xyz = _pvec("ee_offset_right_xyz")
        self._ee_off_right_rpy_deg = _pvec("ee_offset_right_rpy_deg")
        self._ee_off_left_xyz = _pvec("ee_offset_left_xyz")
        self._ee_off_left_rpy_deg = _pvec("ee_offset_left_rpy_deg")

        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()
        self._last_q_target = np.zeros(14, dtype=float)
        self.arms_enabled = False
        # arm_sdk-Gewicht + Rampenzustand: off -> ramp_up -> on -> ramp_down -> off.
        # Waehrend ramp_down publiziert main_loop weiter (letzte Sollpose), damit
        # die Uebergabe an die Roboter-eigene Steuerung weich ist.
        self._arm_sdk_weight = 0.0
        self._weight_state = "off"
        # E-Stop-Latch: nach /g1pilot/emergency_stop=true verstummt der Node
        # SOFORT (eine letzte Nachricht mit Weight=0 + kp/kd=0 -> Arme schlaff,
        # Damp() faengt den Rest). Quittiert wird ueber /g1pilot/start --
        # dieselbe Semantik wie robot_stopped im loco_client.
        self.estop_active = False
        # Slack-Latch: Arme werden gedaempft-drehmomentfrei gehalten (Weight=1,
        # kp=0), OHNE Rueckgabe an den Onboard-Regler. Bleibt nach dem
        # Quittieren (START) aktiv, bis ENABLE MANIPULATION die Kontrolle
        # bewusst zurueckholt.
        self._arm_slack = False
        self.homing_active = False
        self.homing_reached = False
        self.homing_tolerance = 0.02
        self._last_left_goal_raw = None
        self._last_right_goal_raw = None
        self._goal_left_filt = None
        self._goal_right_filt = None
        self._reset_after_home = False
        self._initialized = False

        # Positionsspeicher (plan-execute, siehe g1pilot/SCENE_BRIDGE.md
        # Abschnitt 9): Endpunkt wird gespeichert, die Bahn dorthin JEDES MAL
        # neu geplant (arm_planner.plan_arms_joint_path), weil Startpose und
        # Umgebung sich seit dem Speichern geaendert haben koennen.
        self._pose_store = PoseStore()
        self._planned_motion_active = False
        # Gleichzeitige Ausfuehrung: EINE geteilte Wegpunktliste ueber die
        # geplanten Arme (7 DOF je Seite, in _planned_sides-Reihenfolge
        # konkateniert). Ein gemeinsamer Index -> beide Arme ruecken synchron
        # weiter (siehe main_loop). _planned_sides == [] bedeutet: keine
        # Arm-Bewegung (z.B. nur Handposition wiederherstellen).
        self._planned_sides = []                      # z.B. ["left","right"] oder ["right"]
        self._planned_waypoints = None                # Liste np.ndarray(7*len(sides))
        self._planned_wp_idx = 0
        self._planned_pose_name = None
        self._planning_thread = None
        # Zuletzt empfangener Hand-Ist-Zustand (6 Inspire-DOF je Hand, 0..1000)
        # von der inspire-Bridge (siehe manipulation/inspire_ftp/bridge.py) --
        # nur zum SPEICHERN der Handposition. None = Bridge liefert (noch) nichts.
        self._hand_state = {"left": None, "right": None}
        # Hand-Ziele, die beim START der geplanten Armbewegung an die Bridge
        # gehen (damit Finger und Arme gemeinsam losfahren). Bei reiner
        # Handposition (ohne Arme) werden sie sofort gesendet.
        self._pending_hand_goals = {}
        self._plan_result = DataBuffer()               # Thread-sicherer Handoff Planer -> main_loop
        self._plan_result_seen = None
        # Generations-Zaehler: jeder POSE ANFAHREN erhoeht ihn, jeder Abbruch
        # (E-Stop/DISABLE/Marker/HOMING/WALK/ABBRECHEN) ebenfalls. Ein Planungs-
        # Ergebnis wird NUR aktiviert, wenn seine Generation noch die aktuelle
        # ist -- so kann ein Marker-Griff (oder E-Stop) WAEHREND einer noch
        # laufenden Planung deren spaeteres Ergebnis zuverlaessig entwerten,
        # ohne dass es schon vorliegen muesste (race-frei).
        self._plan_gen = 0

        self._T_off_right_static = self._mk_static_T(self._ee_off_right_xyz, self._ee_off_right_rpy_deg)
        self._T_off_left_static = self._mk_static_T(self._ee_off_left_xyz, self._ee_off_left_rpy_deg)
        self._T_off_right_auto = None
        self._T_off_left_auto = None
        self._auto_done_right = False
        self._auto_done_left = False

        self.ik_solver = G1IKSolver(debug=False)
        if hasattr(self.ik_solver, "set_orientation_mode"):
            self.ik_solver.set_orientation_mode(self.ik_orientation_mode)

        # Kartesisches Speedlimit: ueberwachte Punkte = Hand-TCPs + Ellbogen
        # (der Ellbogen kann bei reiner Schulterdrehung schneller sein als die
        # Hand). Eigenes pin.Data, damit die FK hier den Solver-Zustand nicht
        # anfasst.
        self._speed_data = pin.Data(self.ik_solver.model)
        self._speed_fids = [f for f in (self.ik_solver._fid_left,
                                        self.ik_solver._fid_right) if f is not None]
        for link in ("left_elbow_link", "right_elbow_link"):
            try:
                self._speed_fids.append(self.ik_solver.model.getFrameId(link))
            except Exception:
                pass
        if self.self_collision_gate and not getattr(self.ik_solver, "_gate_ready", False):
            self.get_logger().warn(
                "Selbstkollisions-Gate NICHT verfuegbar (s. IK-Log) -- "
                "Arm-Kommandos werden ungeprueft gesendet!")
        self._gate_last_warn = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Home-Pose je Arm (7 DOF, Reihenfolge:
        #   [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
        #    wrist_roll, wrist_pitch, wrist_yaw]).
        # Natuerliche Ruhepose: Arme leicht vor (pitch +), leicht seitlich aus
        # (roll links + / rechts -, gespiegelt!), Ellbogen leicht gebeugt (elbow +).
        # Als Parameter -> live tunebar via --ros-args -p home_left:="[...]".
        self.declare_parameter("home_left",  [0.3,  0.2, 0.0, 0.5, 0.0, 0.0, 0.0])
        self.declare_parameter("home_right", [0.3, -0.2, 0.0, 0.5, 0.0, 0.0, 0.0])
        self.home_left  = np.array(self.get_parameter("home_left").value,  dtype=float)
        self.home_right = np.array(self.get_parameter("home_right").value, dtype=float)

        # Ruhe-Pose fuer die IK-Nullraum-Regularisierung = Home-Pose. Haelt den
        # redundanten (7. DOF) Ellbogen-Swivel natuerlich -> der Arm faellt beim
        # Marker-Ziehen nicht in den zum Koerper gefalteten IK-Zweig.
        if hasattr(self.ik_solver, "set_rest_posture"):
            self.ik_solver.set_rest_posture("left",  self.home_left)
            self.ik_solver.set_rest_posture("right", self.home_right)
        self.ik_solver.null_space_gain = float(self.get_parameter("ik_null_space_gain").value)

        # WALK-Pose: waehrend WALK haelt der arm_controller die Arme HIER (= Policy-
        # Default-Armpose des g1_wholebody-Policy). Headless validiert: die Lauf-Policy
        # laeuft mit fixen Armen NUR stabil, wenn sie nahe dieser Pose sind (Arme unten
        # am Koerper -> Sturz). Bei BALANCING sind die Arme wieder frei (rviz/Marker).
        self.declare_parameter("walk_left",  [0.35,  0.18, 0.0, 0.87, 0.0, 0.0, 0.0])
        self.declare_parameter("walk_right", [0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0])
        self.walk_left  = np.array(self.get_parameter("walk_left").value,  dtype=float)
        self.walk_right = np.array(self.get_parameter("walk_right").value, dtype=float)
        self.walk_mode = False
        # Sobald die Arme die Lauf-Pose erreicht haben, meldet der Controller das per
        # /g1pilot/arms/walk_ready -> loco_sim laeuft erst DANN los (Arme aufgeraeumt).
        # Toleranz etwas lockerer als die Homing-Toleranz (die Haltepose muss nicht
        # exakt sitzen, nur "im Wesentlichen aufgeraeumt").
        self.declare_parameter("walk_ready_tolerance", 0.05)
        self.walk_ready_tolerance = float(self.get_parameter("walk_ready_tolerance").value)
        self._walk_ready_sent = False

        self.left_workspace_publisher = self.create_publisher(Marker, '/g1pilot/workspace/left', 10)
        self.right_workspace_publisher = self.create_publisher(Marker, '/g1pilot/workspace/right', 10)
        self.walk_ready_publisher = self.create_publisher(Bool, '/g1pilot/arms/walk_ready', 1)


        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(PoseStamped, "/g1pilot/hand_goal/right", self._right_goal_callback, 10)
        self.create_subscription(PoseStamped, "/g1pilot/hand_goal/left", self._left_goal_callback, 10)
        self.create_subscription(Bool, "/g1pilot/arms/enabled", self._arms_controlled_callback, 10)
        self.create_subscription(Bool, "/g1pilot/arms/home", self._homming_callback, 10)
        # Loco-Zustand: bei WALK Arme in die Lauf-Pose, bei BALANCING wieder frei.
        self.create_subscription(Bool, "/g1pilot/start_walking", self._on_walk_mode, 10)
        self.create_subscription(Bool, "/g1pilot/start_balancing", self._on_balance_mode, 10)
        # E-Stop: sofort verstummen + Arme schlaff schalten (Weight=0, kp/kd=0).
        # /g1pilot/start quittiert den Latch (wie robot_stopped im loco_client).
        self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_emergency_stop, 10)
        self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)

        # Umgebungs-Objekte (Hindernisse + Greif-Objekte), siehe scene_bridge.py.
        # TRANSIENT_LOCAL, damit ein spaeter gestarteter arm_controller den
        # zuletzt veroeffentlichten Stand sofort bekommt (statt bis zum
        # naechsten scene_bridge-Publish-Tick auf leere Umgebung zu laufen).
        qos_scene = QoSProfile(depth=1)
        qos_scene.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(MarkerArray, "/scene_markers", self._on_scene_markers, qos_scene)

        # Positionsspeicher (Streamdeck-Buttons, siehe teleoperation/ui_interface.py).
        self.create_subscription(String, "/g1pilot/pose_store/save", self._on_pose_save, 10)
        self.create_subscription(String, "/g1pilot/pose_store/goto", self._on_pose_goto, 10)
        self.create_subscription(Bool, "/g1pilot/pose_store/cancel", self._on_pose_cancel, 10)

        # Hand-Ist-Zustand von der inspire-Bridge (nur zum SPEICHERN der
        # Handposition) und Hand-Ziel zurueck (zum Wiederherstellen). Laeuft die
        # Bridge nicht, bleibt _hand_state None -> Handposition wird uebersprungen.
        self.create_subscription(
            Float32MultiArray, "/g1pilot/hand_state/left",
            lambda m: self._on_hand_state("left", m), 10)
        self.create_subscription(
            Float32MultiArray, "/g1pilot/hand_state/right",
            lambda m: self._on_hand_state("right", m), 10)
        self.pub_hand_goal = {
            "left": self.create_publisher(Float32MultiArray, "/g1pilot/hand_goal/left", 10),
            "right": self.create_publisher(Float32MultiArray, "/g1pilot/hand_goal/right", 10),
        }

        self._init_robot_interface()

        self._last_tick_time = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self.main_loop)

    def _arm_gravity_tau(self, q14):
        """Schwerkraft-Drehmomente [Nm] fuer die 14 Arm-Gelenke bei der
        Konfiguration q14 (7 links + 7 rechts) aus dem Pinocchio-Modell."""
        tau = np.zeros(14, dtype=float)
        if not bool(self.get_parameter("gravity_comp").value):
            return tau
        try:
            ik = self.ik_solver
            q_full = pin.neutral(ik.model)
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[i])
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[7 + i])
            g = pin.computeGeneralizedGravity(ik.model, ik.data, q_full)
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                tau[i] = float(g[ik._name_to_v_index[ik._ros_joint_names[arm_i]]])
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                tau[7 + i] = float(g[ik._name_to_v_index[ik._ros_joint_names[arm_i]]])
            np.clip(tau, -20.0, 20.0, out=tau)
        except Exception:
            tau[:] = 0.0
        return tau

    def _mk_static_T(self, xyz, rpy_deg):
        """
        Create a fixed SE3 transform from XYZ translation and RPY rotation in degrees.

        Parameters
        ----------
        xyz : array-like of float
            Translation vector [x, y, z] in meters.
        rpy_deg : array-like of float
            Roll, pitch, yaw angles in degrees.

        Returns
        -------
        pinocchio.SE3
            Homogeneous transform combining translation and rotation.
        """

        rpy = np.radians(np.array(rpy_deg, dtype=float))
        R = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
        return SE3(R, np.array(xyz, dtype=float))
    
    def _goal_error(self, side: str, T_goal: SE3):
        """
        Compute position and orientation error between current and target end-effector pose.

        Parameters
        ----------
        side : str
            Arm identifier ('left' or 'right').
        T_goal : SE3
            Desired end-effector target pose.

        Returns
        -------
        tuple(float, float)
            (position_error_m, orientation_error_rad)
        """

        M_cur = self._fk_current_ee(side)
        if M_cur is None or T_goal is None:
            return None, None
        dp = float(np.linalg.norm(T_goal.translation - M_cur.translation))
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal.rotation)
        ang = 2.0 * math.atan2(
            math.sqrt(dq.x*dq.x + dq.y*dq.y + dq.z*dq.z),
            abs(dq.w)
        )
        return dp, ang

    def _lowpass_goal(self, T_prev: SE3, T_new: SE3, alpha: float) -> SE3:
        """
        Apply exponential smoothing between previous and new SE3 goals.

        Parameters
        ----------
        T_prev : SE3
            Previous goal pose.
        T_new : SE3
            New goal pose.
        alpha : float
            Low-pass coefficient (0.0–1.0).

        Returns
        -------
        SE3
            Smoothed goal transform.
        """

        if T_prev is None:
            return T_new
        p = (1.0 - alpha) * T_prev.translation + alpha * T_new.translation
        q0 = _mat_to_quat_wxyz(T_prev.rotation)
        q1 = _mat_to_quat_wxyz(T_new.rotation)
        qf = (1 - alpha) * q0 + alpha * q1
        qf = qf / np.linalg.norm(qf)
        Rf = _quat_wxyz_to_matrix(qf)
        return SE3(Rf, p)

    def _limit_ori_step(self, R_cur: np.ndarray, R_des: np.ndarray, max_step: float) -> np.ndarray:
        """
        Limit the angular step between current and desired rotation matrices.

        Parameters
        ----------
        R_cur : np.ndarray
            Current 3×3 rotation matrix.
        R_des : np.ndarray
            Desired 3×3 rotation matrix.
        max_step : float
            Maximum angular step (rad).

        Returns
        -------
        np.ndarray
            Limited rotation matrix.
        """

        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        nrm = float(np.linalg.norm(aa))
        if nrm <= 1e-12 or nrm <= max_step:
            return R_des
        aa_lim = aa * (max_step / nrm)
        return R_cur @ pin.exp3(aa_lim)

    def _fk_current_ee(self, side: str):
        """
        Compute the current end-effector pose (SE3) for the given arm using FK.

        Parameters
        ----------
        side : str
            'left' or 'right'.

        Returns
        -------
        SE3 or None
            Forward kinematics result for the end-effector.
        """

        try:
            q_full = pin.neutral(self.ik_solver.model)
            cur_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()
            for jid_idx, ros_name in enumerate(self.ik_solver._ros_joint_names):
                if ros_name in self.ik_solver._name_to_q_index:
                    q_full[self.ik_solver._name_to_q_index[ros_name]] = float(cur_all[jid_idx])
            pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            fid = self.ik_solver._fid_right if side == 'right' else self.ik_solver._fid_left
            if fid is None:
                return None
            return self.ik_solver.data.oMf[fid]
        except Exception:
            return None

    def _gate_auto_calibration(self, T_goal_in: SE3, side: str):
        """
        Check if the current end-effector pose is close enough to the incoming goal
        to perform automatic end-effector calibration.

        Parameters
        ----------
        T_goal_in : SE3
            Incoming target pose before any static offset is applied.
        side : str
            Arm identifier ('left' or 'right').

        Returns
        -------
        SE3 or None
            Current SE3 pose if within calibration threshold, otherwise None.
        """

        M_cur = self._fk_current_ee(side)
        if M_cur is None:
            return None
        dp = np.linalg.norm(T_goal_in.translation - M_cur.translation)
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal_in.rotation)
        ang = 2 * math.atan2(np.linalg.norm([dq.x, dq.y, dq.z]), abs(dq.w))
        if dp < 0.05 and ang < math.radians(12.0):
            return M_cur
        return None

    def _apply_offsets_and_filters(self, side: str, T_goal_input: SE3):
        """
        Apply static and auto-calibrated offsets to an incoming goal and filter it.

        This function handles end-effector calibration (static + automatic),
        goal smoothing, and orientation step limitation.

        Parameters
        ----------
        side : str
            Arm identifier ('left' or 'right').
        T_goal_input : SE3
            Raw goal transform.

        Returns
        -------
        SE3
            Adjusted and filtered goal for IK solver.
        """

        T_static = self._T_off_right_static if side == 'right' else self._T_off_left_static
        T_auto = self._T_off_right_auto if side == 'right' else self._T_off_left_auto
        auto_done = self._auto_done_right if side == 'right' else self._auto_done_left

        if self.ee_auto_calibrate and not auto_done:
            M_cur_ok = self._gate_auto_calibration(T_goal_input, side)
            if M_cur_ok is not None:
                T_pre = T_goal_input * T_static
                T_auto_new = T_pre.inverse() * M_cur_ok
                if side == 'right':
                    self._T_off_right_auto = T_auto_new; self._auto_done_right = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated right: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                else:
                    self._T_off_left_auto = T_auto_new; self._auto_done_left = True
                    t = T_auto_new.translation
                    self.get_logger().info(f"[IK] auto-calibrated left: d=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f})")
                T_auto = T_auto_new

        T_raw = T_goal_input * T_static * (T_auto if T_auto is not None else SE3.Identity())

        if side == 'right':
            self._goal_right_filt = self._lowpass_goal(self._goal_right_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_right_filt
        else:
            self._goal_left_filt = self._lowpass_goal(self._goal_left_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_left_filt

        M_cur = self._fk_current_ee(side)
        if (M_cur is not None) and (T_use is not None):
            R_lim = self._limit_ori_step(M_cur.rotation, T_use.rotation, self.ik_max_ori_step_rad)
            T_use = SE3(R_lim, T_use.translation.copy())

        return T_use

    def _init_robot_interface(self):
        """Initialize DDS interface for robot communication."""

        init_dds(self.interface, self.get_logger())

        self.lowstate_subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.lowstate_subscriber.Init()

        self.subscribe_thread = threading.Thread(
            target=self._subscribe_motor_state, daemon=True
        )
        self.subscribe_thread.start()

        self.lowcmd_publisher = ChannelPublisher('rt/arm_sdk', LowCmd_)
        self.lowcmd_publisher.Init()

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data from DDS...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        self.all_motor_q = self.get_current_motor_q()

        # PD-Gains. Defaults = Sim-Tuning (kd hoch, weil die MuJoCo-Gelenke
        # sonst unterdaempft um die Sollpose schwingen). Das REAL-Bringup
        # (bringup_real -> manipulation_launcher) ueberschreibt auf die im
        # offiziellen Unitree-g1_arm7_sdk-Beispiel erprobten Werte kp=60,
        # kd=1.5 -- NICHT die Sim-Werte auf echte Motoren loslassen.
        def _gain(name, default):
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
            return float(self.get_parameter(name).value)

        self.kp_low   = _gain("kp_low",   150.0); self.kd_low   = _gain("kd_low",   12.0)
        self.kp_wrist = _gain("kp_wrist",  40.0); self.kd_wrist = _gain("kd_wrist",  4.0)

        # NUR die 14 Arm-Gelenke (15..28) + Weight (29) werden je beschrieben --
        # exakt wie im offiziellen Unitree-g1_arm7_sdk-Beispiel. Alle anderen
        # Eintraege werden hier einmalig explizit inert gesetzt und danach NIE
        # angefasst: Beine gehoeren nicht in rt/arm_sdk, und die Taille (12-14)
        # ist via arm_sdk real ansteuerbar -- ein Kommando dort wuerde gegen
        # den Unitree-Loco-Controller kaempfen. (Frueher schrieb
        # _hold_non_arm_joints Beine+Taille mit kp=300 in die Message.)
        arm_vals = {m.value for m in G1_29_JointArmIndex}
        for jid in G1_29_JointIndex:
            if jid.value in arm_vals:
                continue
            mc = self.msg.motor_cmd[jid]
            mc.mode = 0
            mc.q = 0.0; mc.dq = 0.0; mc.tau = 0.0; mc.kp = 0.0; mc.kd = 0.0

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            self.msg.motor_cmd[jid].q = float(self.all_motor_q[jid.value])

        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self._initialized = True

    def _subscribe_motor_state(self):
        """
        Thread loop continuously reading motor state messages from DDS.

        Runs as a daemon thread to asynchronously update internal motor states
        and store the latest LowState message in `lowstate_buffer`.

        Notes
        -----
        - This method blocks indefinitely while ROS 2 is running.
        - Updates both joint positions and velocities for each motor.
        """

        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def get_mode_machine(self) -> int:
        """
        Get the current robot mode from the latest LowState message.

        Returns
        -------
        int
            Current machine mode identifier, or 0 if no data available.
        """

        msg = self.lowstate_buffer.GetData()
        return getattr(msg, "mode_machine", 0) if msg is not None else 0

    def get_current_motor_q(self) -> np.ndarray:
        """
        Retrieve the current joint positions (q) for all robot joints.

        Returns
        -------
        np.ndarray
            Array of joint positions (rad) ordered according to `G1_29_JointIndex`.
        """

        msg = self.lowstate_buffer.GetData()
        return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)

    def _assemble_full_from_last(self) -> np.ndarray:
        """
        Build a complete 29-DOF joint configuration vector from the latest arm targets.

        Combines left and right 7-DOF joint targets into a full-body vector.

        Returns
        -------
        np.ndarray
            Full 29-element joint configuration array.
        """

        full = np.zeros(29, dtype=float)
        for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[i]
        for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[7 + i]
        return full

    def _full_config_with_arms(self, q14: np.ndarray) -> np.ndarray:
        """29-DOF-Konfiguration: Arme aus q14, Rest (Beine/Taille) aus der
        Messung (real) bzw. neutral (Sim)."""
        if self.use_robot:
            try:
                full = self.get_current_motor_q()
            except Exception:
                full = np.zeros(29, dtype=float)
        else:
            full = np.zeros(29, dtype=float)
        for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
            full[jidx] = q14[i]
        for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
            full[jidx] = q14[7 + i]
        return full

    def _arm_points(self, q14: np.ndarray) -> np.ndarray:
        """Positionen der ueberwachten Punkte (Hand-TCPs + Ellbogen) fuer die
        Arm-Konfiguration q14 (Nicht-Arm-Gelenke neutral -- fuer die
        RELATIVE Verschiebung zwischen zwei Ticks ausreichend)."""
        ik = self.ik_solver
        q_full = pin.neutral(ik.model)
        for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
            q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[i])
        for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
            q_full[ik._name_to_q_index[ik._ros_joint_names[arm_i]]] = float(q14[7 + i])
        pin.forwardKinematics(ik.model, self._speed_data, q_full)
        pin.updateFramePlacements(ik.model, self._speed_data)
        return np.array([self._speed_data.oMf[f].translation for f in self._speed_fids])

    def _limit_cartesian_speed(self, q_prev: np.ndarray, q_new: np.ndarray,
                               dt: float) -> np.ndarray:
        """Schritt q_prev->q_new so skalieren, dass kein ueberwachter Punkt
        schneller als ee_velocity_limit [m/s] faehrt (ISO 10218-1 reduced
        speed). Skaliert den GESAMTEN Gelenkschritt richtungserhaltend."""
        if self.ee_velocity_limit <= 0.0 or not self._speed_fids:
            return q_new
        dq = q_new - q_prev
        if float(np.max(np.abs(dq))) < 1e-9:
            return q_new
        try:
            disp = float(np.max(np.linalg.norm(
                self._arm_points(q_new) - self._arm_points(q_prev), axis=1)))
        except Exception:
            return q_new
        lim = self.ee_velocity_limit * dt
        if disp <= lim or disp < 1e-12:
            return q_new
        return q_prev + dq * (lim / disp)

    def _in_collision(self, full: np.ndarray, hard: bool = False) -> bool:
        """Kombinierter Kollisions-Check: Selbstkollision (self_collision_gate)
        UND/ODER Umgebungskollision (environment_collision_gate, Hindernisse +
        Greif-ACM -- siehe ik_solver.environment_command_in_collision). Zwei
        getrennte Schalter: ein Operator kann z.B. nur die Umgebungspruefung
        testweise abschalten, ohne die Selbstkollisionspruefung zu verlieren.
        (ik_solver.arm_command_in_collision gibt selbst False zurueck, wenn
        das Gate nicht bereit ist -- kein doppelter Readiness-Check noetig.)"""
        ik = self.ik_solver
        if self.self_collision_gate and ik.arm_command_in_collision(full, hard=hard):
            return True
        if self.environment_collision_gate and ik.environment_command_in_collision(full, hard=hard):
            return True
        return False

    def _apply_collision_gate(self, q_prev: np.ndarray, q_new: np.ndarray) -> np.ndarray:
        """Kollisions-Gate (Selbst + Umgebung) auf dem KOMMANDIERTEN Schritt:
          * Kandidat frei                      -> senden.
          * Kandidat in HARTER Kollision       -> halten (q_prev).
          * Kandidat im Margenband, aktueller Zustand auch -> zulassen
            (Rueckzug aus dem Band muss moeglich sein; Tempo ist ohnehin
            durch das kartesische Limit gedeckelt).
          * Kandidat faehrt NEU ins Band       -> halten (q_prev).

        Laeuft unconditional fuer JEDEN q_target (Marker-Servoing UND geplante
        Bewegung) -- auch eine bereits kollisionsfrei GEPLANTE Bahn bekommt so
        denselben Laufzeit-Sicherheitsnetz-Check pro Tick (falls sich die
        Umgebung seit dem Planen bewegt hat)."""
        if float(np.max(np.abs(q_new - q_prev))) < 1e-9:
            return q_new
        try:
            full_new = self._full_config_with_arms(q_new)
            if not self._in_collision(full_new):
                return q_new
            if not self._in_collision(full_new, hard=True):
                if self._in_collision(self._full_config_with_arms(q_prev)):
                    return q_new     # schon im Band -> langsames Herausfahren erlaubt
        except Exception as e:
            self.get_logger().warn(f"Kollisions-Gate-Fehler: {e} -- halte Pose.")
            return q_prev
        now = time.time()
        if now - self._gate_last_warn > 1.0:
            self._gate_last_warn = now
            self.get_logger().warn(
                "Kollisions-Gate: Zielbewegung wuerde in Koerper/Hindernis "
                "fahren -- Arm haelt an (Marker zurueckziehen).")
        return q_prev

    def _advance_arm_weight(self, dt: float) -> float:
        """arm_sdk-Gewicht (motor_cmd[29].q) einen Tick weiterfahren.

        ramp_up: 0->1 in arm_weight_ramp_up_s (0.0 = sofort 1.0, Sim-Verhalten).
        ramp_down: 1->0 in arm_weight_ramp_down_s, danach Zustand off.
        """
        if self._weight_state == "ramp_up":
            ramp = float(self.get_parameter("arm_weight_ramp_up_s").value)
            if ramp <= 0.0:
                self._arm_sdk_weight = 1.0
            else:
                self._arm_sdk_weight = min(1.0, self._arm_sdk_weight + dt / ramp)
            if self._arm_sdk_weight >= 1.0:
                self._weight_state = "on"
        elif self._weight_state == "ramp_down":
            ramp = float(self.get_parameter("arm_weight_ramp_down_s").value)
            if ramp <= 0.0:
                self._arm_sdk_weight = 0.0
            else:
                self._arm_sdk_weight = max(0.0, self._arm_sdk_weight - dt / ramp)
            if self._arm_sdk_weight <= 0.0:
                self._weight_state = "off"
        elif self._weight_state == "on":
            self._arm_sdk_weight = 1.0
        else:
            self._arm_sdk_weight = 0.0
        return self._arm_sdk_weight

    def _publish_weight_ramp_down(self):
        """Nach dem Disable: letzte Arm-Sollwerte weiter publizieren, waehrend das
        arm_sdk-Gewicht 1->0 faehrt. self.msg enthaelt die zuletzt geschriebenen
        Arm-Kommandos noch -- nur Gewicht + CRC aktualisieren."""
        if not self.use_robot:
            self._weight_state = "off"
            self._arm_sdk_weight = 0.0
            return
        w = self._advance_arm_weight(self._compute_dt())
        try:
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = float(w)
        except Exception:
            pass
        self.msg.crc = self.crc.Crc(self.msg)
        self.lowcmd_publisher.Write(self.msg)
        if self._weight_state == "off":
            self.get_logger().info("arm_sdk-Gewicht auf 0 -- Arme an die "
                                   "Roboter-Steuerung uebergeben.")

    def _publish_arm_slack(self):
        """Arme drehmomentfrei / leicht gedaempft halten -- OHNE die Kontrolle
        an den Unitree-Onboard-Regler zurueckzugeben.

        KRITISCH: das arm_sdk-Gewicht bleibt hier auf 1. Setzt man es auf 0
        (frueheres E-Stop-Verhalten!), uebernimmt der Onboard-Regler die Arme
        und faehrt sie AKTIV mit voller Geschwindigkeit in seine Default-Pose
        (sieht aus wie ein Sprung in die Home-Pose). Mit Gewicht=1 behaelt
        arm_sdk die Autoritaet: kp=0 -> keine Positionshaltung, tau=0 -> kein
        Feedforward, kd klein -> die Arme sacken GEDAEMPFT (an Seilen
        abgefangen). Muss WEITER gesendet werden, sonst greift der arm_sdk-
        Watchdog und der Onboard-Regler schnappt doch noch zu."""
        if not self.use_robot or not getattr(self, "_initialized", False):
            return
        try:
            cur = self.get_current_motor_q()
        except Exception:
            cur = None
        try:
            self.msg.mode_machine = self.get_mode_machine()
            for jid in G1_29_JointArmIndex:
                mc = self.msg.motor_cmd[jid]
                mc.mode = 1
                mc.kp = 0.0
                mc.kd = self.estop_arm_kd
                mc.tau = 0.0
                mc.dq = 0.0
                if cur is not None:
                    mc.q = float(cur[jid.value])   # bei kp=0 ohne Wirkung, sauber
            # Weight = 1: arm_sdk behaelt die Autoritaet (KEIN Onboard-Snap).
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
        except Exception as e:
            self.get_logger().error(f"Arm-Slack-Nachricht fehlgeschlagen: {e}")

    def _on_emergency_stop(self, msg: Bool):
        """E-STOP: Arme drehmomentfrei/gedaempft, OHNE Onboard-Uebergabe.

        Der Slack-Zustand wird ab jetzt in jedem Tick weiter gesendet
        (_publish_arm_slack) -- kein Handover-Fenster, in dem der Onboard-
        Regler die Arme in die Home-Pose reissen koennte. Der Latch
        (estop_active) blockiert ENABLE, bis via /g1pilot/start quittiert
        wird; die Arme bleiben schlaff, bis ENABLE die Kontrolle bewusst
        zurueckholt. Beine/Koerper werden separat von loco_client gedaempft
        (Damp)."""
        if not msg.data or self.estop_active:
            return
        self.estop_active = True
        self._arm_slack = True
        self.arms_enabled = False
        self.homing_active = False
        self.homing_reached = False
        self.walk_mode = False
        self._abort_planned_motion()   # keine Ueberraschungsbewegung nach dem Quittieren
        self._weight_state = "off"
        self._arm_sdk_weight = 1.0     # Autoritaet behalten (kein Onboard-Snap)
        self._publish_arm_slack()
        self.get_logger().warn("EMERGENCY STOP: Arme drehmomentfrei/gedaempft "
                               "(arm_sdk behaelt Autoritaet, KEIN Sprung in "
                               "Home). Quittieren: START.")

    def _on_start(self, msg: Bool):
        """START (Streamdeck) quittiert den E-Stop-Latch. Die Arme bleiben
        schlaff (_arm_slack) und werden weiter gedaempft gehalten, bis ENABLE
        MANIPULATION die Kontrolle bewusst zurueckholt -- NICHT an den
        Onboard-Regler zurueckgeben (der wuerde sie posieren)."""
        if msg.data and self.estop_active:
            self.estop_active = False
            self.get_logger().info("E-Stop quittiert (START) -- Arme bleiben "
                                   "gedaempft schlaff bis ENABLE MANIPULATION.")

    def _arms_controlled_callback(self, msg: Bool):
        """
        ROS 2 callback to enable or disable arm control.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            True to enable arm control, False to disable.

        Behavior
        --------
        - On enable: stores current joint state as last target.
        - On disable: stops sending active motion commands.
        """

        if msg.data and self.estop_active:
            self.get_logger().warn("ENABLE ignoriert: E-Stop aktiv -- erst mit "
                                   "START quittieren.")
            return
        self.arms_enabled = msg.data
        # Reste einer geplanten Bewegung IMMER verwerfen (Enable UND Disable):
        # sonst feuerte ein waehrend disabled fertig gewordener Plan beim
        # naechsten Enable als Ueberraschung los -- gleiche Vorsicht wie beim
        # Homing unten. (Auch beim Disable, damit ein noch laufender Planungs-
        # Thread sein Ergebnis nicht spaeter aktiviert.)
        self._abort_planned_motion()
        if self.arms_enabled:
            # Slack-Latch aufheben: ENABLE holt die Kontrolle bewusst zurueck.
            self._arm_slack = False
            try:
                cur = self.get_current_motor_q()
                left = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right= [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                pass
            # Reste einer frueheren Session verwerfen: alte IK-Ziele wuerden den
            # Arm beim Re-Enable SOFORT auf ein altes Marker-Ziel losfahren
            # lassen, ein altes homing_reached wuerde ihn zur Home-Pose treiben.
            # Stattdessen: HALTEN an der aktuellen Stellung, bis der Nutzer
            # einen Marker zieht.
            self.homing_active = False
            self.homing_reached = False
            self._align_ik_to_config(self._last_q_target[0:7],
                                     self._last_q_target[7:14])
            # Gewicht sanft hochfahren (bzw. Abwaertsrampe nahtlos umkehren).
            if self._weight_state != "on":
                self._weight_state = "ramp_up"
            self.get_logger().info("Arm ENABLED.")
        else:
            ramp_down = float(self.get_parameter("arm_weight_ramp_down_s").value)
            if (ramp_down > 0.0
                    and self._weight_state in ("ramp_up", "on")
                    and self._arm_sdk_weight > 0.0):
                self._weight_state = "ramp_down"
            else:
                # ramp_down<=0 (Sim): sofort verstummen wie bisher -- KEINE
                # w=0-Nachricht senden (die MuJoCo-Bridge haelt die Arme mit
                # dem zuletzt empfangenen Kommando; eine w=0-Nachricht wuerde
                # sie dort schlaff schalten = Verhaltensaenderung).
                self._weight_state = "off"
                self._arm_sdk_weight = 0.0
            self.get_logger().info("Arm DISABLED")

    def _homming_callback(self, msg: Bool):
        """
        ROS 2 callback that triggers the homing sequence for both arms.

        Parameters
        ----------
        msg : std_msgs.msg.Bool
            If True, initiates homing to predefined joint targets.

        Notes
        -----
        - Clears IK goals before starting the homing motion.
        - Sets internal flags for homing management.
        """

        if msg.data:
            # Nur bei aktiver Manipulation annehmen. Frueher wurde ein Homing-
            # Klick im disabled-Zustand GESPEICHERT und feuerte dann beim
            # naechsten Enable als Ueberraschungsbewegung.
            if not self.arms_enabled or self.estop_active:
                self.get_logger().warn("HOMING ignoriert: Manipulation nicht "
                                       "aktiv (erst ENABLE MANIPULATION).")
                return
            self._abort_planned_motion("HOMING gestartet")
            self.get_logger().info("Moving both arms to HOME position.")
            self.homing_active = True
            self.homing_reached = False
            self._reset_after_home = False
            if hasattr(self.ik_solver, "clear_goals"):
                self.ik_solver.clear_goals()

    def _align_ik_to_config(self, left_q, right_q):
        """IK-Ziele auf die gegebene Arm-Konfiguration setzen -> die Arme HALTEN dort,
        bis der Nutzer einen Marker zieht (kein Sprung auf ein altes Ziel)."""
        left_q = np.asarray(left_q, dtype=float); right_q = np.asarray(right_q, dtype=float)
        try:
            if hasattr(self.ik_solver, "clear_goals"):
                self.ik_solver.clear_goals()
            self.ik_solver.set_current_configuration({"left": left_q.copy(), "right": right_q.copy()})
            q_full = pin.neutral(self.ik_solver.model)
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = left_q[i]
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = right_q[i]
            pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            T_left  = self.ik_solver.data.oMf[self.ik_solver._fid_left]
            T_right = self.ik_solver.data.oMf[self.ik_solver._fid_right]
            self._goal_left_filt  = T_left.copy()
            self._goal_right_filt = T_right.copy()
            if hasattr(self.ik_solver, "set_goal"):
                self.ik_solver.set_goal("left",  T_left.copy())
                self.ik_solver.set_goal("right", T_right.copy())
            self._reset_after_home = True
        except Exception as e:
            self.get_logger().warning(f"IK-Align fehlgeschlagen: {e}")

    def _on_walk_mode(self, msg: Bool):
        """WALK: Arme sanft in die Lauf-Pose fahren und dort halten (Marker
        werden waehrend WALK ignoriert), damit die Lauf-Policy stabil bleibt."""
        if not msg.data or self.walk_mode:
            return
        self.walk_mode = True
        self.homing_active = False
        self.homing_reached = False
        self._abort_planned_motion("WALK gestartet")
        self._walk_ready_sent = False    # erst melden, wenn die Lauf-Pose erreicht ist
        self.get_logger().info("WALK-Modus: Arme in Lauf-Pose aufraeumen, dann walk_ready.")

    def _on_balance_mode(self, msg: Bool):
        """BALANCING: Arme wieder freigeben (rviz/Marker). Sie HALTEN ihre aktuelle
        Stellung, bis der Nutzer einen Marker zieht."""
        if not msg.data or not self.walk_mode:
            return
        self.walk_mode = False
        self.homing_active = False
        self.homing_reached = False
        if self._walk_ready_sent:
            self._walk_ready_sent = False
            self.walk_ready_publisher.publish(Bool(data=False))
        self._align_ik_to_config(self._last_q_target[0:7], self._last_q_target[7:14])
        self.get_logger().info("BALANCING-Modus: Arme frei (rviz/Marker).")

    def _transform_pose_to_world(self, ps: PoseStamped) -> PoseStamped:
        """
        Transform an incoming pose message into the world (IK) reference frame.

        Parameters
        ----------
        ps : geometry_msgs.msg.PoseStamped
            Input pose message with arbitrary frame_id.

        Returns
        -------
        geometry_msgs.msg.PoseStamped
            Pose transformed into `self.frame` if possible; original if TF lookup fails.
        """

        if not ps.header.frame_id or ps.header.frame_id == self.frame:
            return ps
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2))
            return do_transform_pose(ps, tf)
        except Exception as e:
            self.get_logger().warning(f"[IK] TF {ps.header.frame_id}->{self.frame} failed: {e}")
            return ps

    def _on_scene_markers(self, msg: MarkerArray):
        """Umgebungs-Objekte (/scene_markers, i.d.R. Frame 'map') EINMAL pro
        Update in den IK-World-Frame (self.frame, Default 'pelvis') transformieren
        und in den IK-Solver einspeisen (sync_environment). Nur EIN TF-Lookup
        pro Update (nicht pro Marker) -- alle Marker teilen sich denselben
        Quell-Frame, do_transform_pose() je Marker ist reine Pose-Arithmetik,
        keine erneute TF-Baum-Abfrage."""
        add_markers = [m for m in msg.markers if m.action == Marker.ADD]
        if not add_markers:
            self.ik_solver.sync_environment([])
            return
        frame_id = add_markers[0].header.frame_id or "map"
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, frame_id, Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(
                f"[scene] TF {frame_id}->{self.frame} fehlgeschlagen: {e} -- "
                f"Umgebungs-Objekte fuer dieses Update uebersprungen.")
            return

        objects = []
        for marker in add_markers:
            try:
                ps = PoseStamped()
                ps.header = marker.header
                ps.pose = marker.pose
                ps_t = do_transform_pose(ps, tf)
                half = sm.local_half_extents_from_marker(marker)
                name, _ = sm.decode_text(marker.text)
                o, p = ps_t.pose.orientation, ps_t.pose.position
                objects.append({
                    "name": name or f"marker_{marker.ns}_{marker.id}",
                    "cls": sm.class_from_ns(marker.ns),
                    "half_extents": half,
                    "pos": [p.x, p.y, p.z],
                    "quat": [o.w, o.x, o.y, o.z],
                })
            except Exception as e:
                self.get_logger().warn(f"[scene] Marker uebersprungen: {e}")
        self.ik_solver.sync_environment(objects)

    # --------------------------------------------------------
    # Positionsspeicher (plan-execute), siehe g1pilot/SCENE_BRIDGE.md Abschnitt 9
    # --------------------------------------------------------

    def _on_hand_state(self, side: str, msg: Float32MultiArray):
        """Aktuellen Fingerzustand (6 Inspire-DOF, 0..1000) von der Bridge
        merken -- ausschliesslich, um ihn beim Speichern der Handposition
        mitnehmen zu koennen."""
        vals = list(msg.data)
        if len(vals) == 6:
            self._hand_state[side] = [float(v) for v in vals]

    def _publish_hand_goals(self, hand_goals: dict) -> None:
        """Gespeicherte Handstellungen an die inspire-Bridge senden (setzt dort
        alle 6 DOF je Hand, siehe bridge.py /g1pilot/hand_goal)."""
        for side, vals in hand_goals.items():
            if vals is None:
                continue
            m = Float32MultiArray()
            m.data = [float(v) for v in vals]
            self.pub_hand_goal[side].publish(m)
            self.get_logger().info(f"Handposition ({side}) wiederhergestellt.")

    def _on_pose_save(self, msg: String):
        """Speichert eine Pose mit AUSWAHL der Komponenten. Nachricht ist
        entweder ein JSON `{"name":..., "components":[...]}` (aus der GUI) oder
        -- rueckwaerts-kompatibel -- ein reiner Name (dann beide Arme). Gueltige
        components: 'left_arm', 'right_arm', 'hand' (beide Haende)."""
        raw = (msg.data or "").strip()
        if not raw:
            self.get_logger().warn("Pose speichern ignoriert: kein Name angegeben.")
            return
        name, components = raw, ["left_arm", "right_arm"]
        if raw.startswith("{"):
            try:
                obj = json.loads(raw)
                name = (obj.get("name") or "").strip()
                components = list(obj.get("components", []))
            except (ValueError, TypeError):
                self.get_logger().warn("Pose speichern ignoriert: ungueltiges JSON.")
                return
        if not name:
            self.get_logger().warn("Pose speichern ignoriert: kein Name angegeben.")
            return

        kwargs = {}
        if "left_arm" in components:
            kwargs["left_arm"] = self._last_q_target[0:7].copy()
        if "right_arm" in components:
            kwargs["right_arm"] = self._last_q_target[7:14].copy()
        if "hand" in components:
            for side in ("left", "right"):
                st = self._hand_state[side]
                if st is not None:
                    kwargs[f"{side}_hand"] = list(st)
            if "left_hand" not in kwargs and "right_hand" not in kwargs:
                self.get_logger().warn(
                    "Handposition ausgewaehlt, aber kein Hand-Zustand verfuegbar "
                    "(inspire-Bridge nicht aktiv?) -- Haende werden ausgelassen.")

        if not kwargs:
            self.get_logger().warn(
                f"Pose '{name}' nicht gespeichert: nichts Speicherbares ausgewaehlt.")
            return
        try:
            self._pose_store.save(name, **kwargs)
            self.get_logger().info(
                f"Pose '{name}' gespeichert ({', '.join(sorted(kwargs))}).")
        except Exception as e:
            self.get_logger().error(f"Pose '{name}' konnte nicht gespeichert werden: {e}")

    def _on_pose_cancel(self, msg: Bool):
        if not msg.data:
            return
        self._abort_planned_motion("POSE ABBRECHEN")

    def _on_pose_goto(self, msg: String):
        name = (msg.data or "").strip()
        if not name:
            return
        if self.estop_active or not self.arms_enabled or self.homing_active or self.walk_mode:
            self.get_logger().warn(
                f"POSE ANFAHREN '{name}' ignoriert: Arme nicht bereit "
                f"(E-Stop quittiert? ENABLE MANIPULATION? nicht am Homen/Laufen?).")
            return
        entry = self._pose_store.get(name)
        if entry is None:
            self.get_logger().warn(f"Pose '{name}' nicht gefunden.")
            return
        if self._planning_thread is not None and self._planning_thread.is_alive():
            self.get_logger().warn("Es laeuft bereits eine Planung -- bitte warten.")
            return

        # Welche Arme sind gespeichert? Reihenfolge fix (links, dann rechts) --
        # das ist die Spaltenreihenfolge im gemeinsamen Gelenkvektor.
        sides = [s for s in ("left", "right") if f"{s}_arm" in entry]
        goals = {s: np.asarray(entry[f"{s}_arm"], dtype=float) for s in sides}
        hand_goals = {s: entry.get(f"{s}_hand") for s in ("left", "right")
                      if f"{s}_hand" in entry}

        if not sides and not hand_goals:
            self.get_logger().warn(f"Pose '{name}' enthaelt nichts zum Anfahren.")
            return

        if not sides:
            # Nur Handposition -- keine Armplanung noetig, sofort senden.
            self._publish_hand_goals(hand_goals)
            self.get_logger().info(f"Pose '{name}': nur Handposition wiederhergestellt.")
            return

        # Hand-Ziele fahren beim START der Armbewegung mit los (gemeinsam).
        self._pending_hand_goals = hand_goals
        self._plan_gen += 1                    # neue Planungs-Generation
        gen = self._plan_gen
        self.get_logger().info(
            f"Plane Weg zu Pose '{name}' ({'+'.join(sides)}) ...")
        self._planning_thread = threading.Thread(
            target=self._plan_pose_worker, args=(name, gen, sides, goals), daemon=True)
        self._planning_thread.start()

    def _plan_pose_worker(self, name: str, gen: int, sides: list, goals: dict):
        """Laeuft in einem Hintergrund-Thread (Planung kann Sekunden dauern) --
        blockt NIE den 250-Hz-Regelkreis. Plant die AUSGEWAEHLTEN Arme
        GEMEINSAM (7 DOF je Seite -> bei beiden Armen ein 14-DOF-Problem), sodass
        Arm-zu-Arm an jedem Zwischenzustand geprueft ist und beide Arme spaeter
        SYNCHRON abgefahren werden koennen (siehe main_loop). Ergebnis (mit
        Generation `gen`) geht per DataBuffer (thread-sicher) an main_loop."""
        try:
            base_current_all = (self.get_current_motor_q() if self.use_robot
                                else self._assemble_full_from_last())
            # Start/Ziel/Limits ueber die Seiten in FESTER Reihenfolge konkatenieren.
            q_start, q_goal, limits = [], [], []
            for side in sides:
                sl = slice(0, 7) if side == "left" else slice(7, 14)
                idx_list = LEFT_JOINT_INDICES_LIST if side == "left" else RIGHT_JOINT_INDICES_LIST
                q_start.append(self._last_q_target[sl].copy())
                q_goal.append(goals[side])
                limits += [JOINT_LIMITS_RAD[i] for i in idx_list]
            q_start = np.concatenate(q_start)
            q_goal = np.concatenate(q_goal)

            path, reason = plan_arms_joint_path(
                self.ik_solver, sides, base_current_all, q_start, q_goal, limits)
            if path is None:
                self._plan_result.SetData(("failed", name, gen, sides, reason))
                self.get_logger().warn(
                    f"Planung zu Pose '{name}' ({'+'.join(sides)}) fehlgeschlagen: {reason}")
                return
            waypoints = shortcut_path(self.ik_solver, sides, base_current_all, path)
            self._plan_result.SetData(("ok", name, gen, sides, waypoints))
            self.get_logger().info(
                f"Planung zu Pose '{name}' erfolgreich ({len(waypoints)} Wegpunkte, "
                f"{'+'.join(sides)}).")
        except Exception as e:
            self._plan_result.SetData(("failed", name, gen, sides, str(e)))
            self.get_logger().error(f"Planung zu Pose '{name}' abgestuerzt: {e}")

    def _poll_plan_result(self):
        """Pro Tick billig pruefen, ob eine Hintergrund-Planung fertig wurde
        (neues Objekt im DataBuffer -- Identitaetsvergleich reicht, SetData
        ersetzt das Objekt bei jedem Aufruf). Aktiviert nur Ergebnisse der
        AKTUELLEN Generation (result[2]); veraltete (durch Abbruch/neue Planung
        invalidierte) werden verworfen."""
        result = self._plan_result.GetData()
        if result is None or result is self._plan_result_seen:
            return
        self._plan_result_seen = result
        if result[2] != self._plan_gen:
            return   # veraltet: abgebrochen oder von einer neueren Planung ueberholt.
        if result[0] != "ok":
            return   # Fehlschlag wurde schon vom Worker geloggt -- Arm haelt einfach.
        # Defense-in-depth: sind die Arme seit dem Planungsstart gesperrt worden
        # (E-Stop, DISABLE, Homing, WALK)? Dann verwerfen. Normalerweise hat ein
        # solcher Wechsel _plan_gen bereits erhoeht (Abbruch); diese Pruefung
        # faengt Restfaelle ab.
        if (self.estop_active or not self.arms_enabled
                or self.homing_active or self.walk_mode):
            self.get_logger().info(
                f"Geplante Bewegung zu Pose '{result[1]}' verworfen "
                f"(Arme nicht mehr bereit -- erneut POSE ANFAHREN).")
            return
        _, name, _gen, sides, waypoints = result
        self._planned_pose_name = name
        self._planned_sides = list(sides)
        self._planned_waypoints = waypoints
        self._planned_wp_idx = 0
        self._planned_motion_active = True
        self.homing_active = False
        self.homing_reached = False
        # Finger fahren gemeinsam mit den Armen los.
        if self._pending_hand_goals:
            self._publish_hand_goals(self._pending_hand_goals)
            self._pending_hand_goals = {}
        self.get_logger().info(f"Fahre geplante Bewegung zu Pose '{name}' ab.")

    def _abort_planned_motion(self, reason: str = "") -> None:
        """Laufende geplante Bewegung abbrechen UND jede noch laufende Planung
        entwerten: durch Erhoehen von _plan_gen wird ein spaeter eintreffendes
        Ergebnis in _poll_plan_result verworfen (race-frei -- das Ergebnis muss
        beim Abbruch noch nicht vorliegen). Wird bei E-Stop, DISABLE, ENABLE,
        HOMING, WALK, Marker-Griff und POSE ABBRECHEN aufgerufen."""
        was_active = self._planned_motion_active
        self._planned_motion_active = False
        self._planned_sides = []
        self._planned_waypoints = None
        self._pending_hand_goals = {}
        self._plan_gen += 1     # invalidiert jedes in-flight/pending Planungs-Ergebnis
        if was_active and reason:
            self.get_logger().info(f"Geplante Bewegung abgebrochen ({reason}).")

    def _right_goal_callback(self, msg: PoseStamped):
        """
        ROS 2 callback for the right-hand end-effector goal.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Desired right-hand pose (can be in any TF frame).

        Behavior
        --------
        - Applies TF transformation to the world frame.
        - Handles homing reset alignment if needed.
        - Applies static and auto-calibration offsets.
        - Updates the IK solver's right-hand goal.
        """

        if self.homing_active:
            return

        if not self.arms_enabled:
            return

        # Mode-Mux (siehe g1pilot/SCENE_BRIDGE.md Abschnitt 8): manueller
        # Marker hat IMMER Vorrang -- ein laufender ODER gerade geplanter Plan
        # (Positionsspeicher) wird sofort verworfen, kein Kaempfen zweier
        # Geschwindigkeitsquellen.
        if self._planned_motion_active or (
                self._planning_thread is not None and self._planning_thread.is_alive()):
            self._abort_planned_motion("Marker bewegt -- manueller Vorrang")

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration({
                "left":  self._last_q_target[0:7].copy(),
                "right": self._last_q_target[7:14].copy()
            })

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_right_goal_raw = T_goal_in
        T_goal_use = self._apply_offsets_and_filters('right', T_goal_in)
        if T_goal_use is not None:
            self.ik_solver.set_goal("right", T_goal_use)


    def _left_goal_callback(self, msg: PoseStamped):
        """
        ROS 2 callback for the left-hand end-effector goal.

        Parameters
        ----------
        msg : geometry_msgs.msg.PoseStamped
            Desired left-hand pose (can be in any TF frame).

        Behavior
        --------
        - Applies TF transformation to the world frame.
        - Handles homing reset alignment if needed.
        - Applies static and auto-calibration offsets.
        - Updates the IK solver's left-hand goal.
        """

        if self.homing_active:
            return

        if not self.arms_enabled:
            return

        # Mode-Mux (siehe g1pilot/SCENE_BRIDGE.md Abschnitt 8): manueller
        # Marker hat IMMER Vorrang -- ein laufender ODER gerade geplanter Plan
        # (Positionsspeicher) wird sofort verworfen, kein Kaempfen zweier
        # Geschwindigkeitsquellen.
        if self._planned_motion_active or (
                self._planning_thread is not None and self._planning_thread.is_alive()):
            self._abort_planned_motion("Marker bewegt -- manueller Vorrang")

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()

            self._goal_left_filt  = None
            self._goal_right_filt = None

            self.ik_solver.set_current_configuration({
                "left":  self._last_q_target[0:7].copy(),
                "right": self._last_q_target[7:14].copy()
            })

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        self._last_left_goal_raw = T_goal_in
        T_goal_use = self._apply_offsets_and_filters('left', T_goal_in)
        if T_goal_use is not None:
            self.ik_solver.set_goal("left", T_goal_use)


    def _compute_dt(self) -> float:
        """
        Compute the elapsed time (Δt) between consecutive main loop cycles.

        Returns
        -------
        float
            Time difference in seconds (clamped to [1e-4, 0.1]).
        """

        now = time.time()
        if self._last_tick_time is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = max(1e-4, min(0.1, now - self._last_tick_time))
        self._last_tick_time = now
        return dt
    
    def _publish_workspace(self, arm):
        marker = Marker()
        marker.header.frame_id = WORKSPACE["frame"]
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.005  
        marker.color = ColorRGBA(r=0.1, g=1.0, b=0.3, a=0.9)

        points = WORKSPACE[arm]

        pts = {k: Point(x=v[0], y=v[1], z=v[2]) for k, v in points.items()}

        edges = [
            # Bottom rectangle
            ("left_bottom_front", "right_bottom_front"),
            ("right_bottom_front", "right_bottom_back"),
            ("right_bottom_back", "left_bottom_back"),
            ("left_bottom_back", "left_bottom_front"),

            # Top rectangle
            ("left_top_front", "right_top_front"),
            ("right_top_front", "right_top_back"),
            ("right_top_back", "left_top_back"),
            ("left_top_back", "left_top_front"),

            # Vertical edges
            ("left_bottom_front", "left_top_front"),
            ("right_bottom_front", "right_top_front"),
            ("left_bottom_back", "left_top_back"),
            ("right_bottom_back", "right_top_back"),
        ]

        for a, b in edges:
            marker.points.append(pts[a])
            marker.points.append(pts[b])

        if arm == "left_arm":
            self.left_workspace_publisher.publish(marker)
        else:
            self.right_workspace_publisher.publish(marker)


    def main_loop(self):
        """
        Main control loop executed at `rate_hz` frequency.

        Core Responsibilities
        ---------------------
        - Update LowState data from DDS.
        - Hold non-arm joints when arms are disabled.
        - Execute homing sequence if active.
        - Update IK solver configuration and compute joint targets.
        - Apply velocity and smoothing limits.
        - Publish joint targets via DDS or /joint_states (simulation).

        Notes
        -----
        - The loop manages both autonomous IK motion and homing control.
        - It automatically synchronizes the IK goals when returning to home.
        """

        self._publish_workspace("left_arm")
        self._publish_workspace("right_arm")

        if not getattr(self, "_initialized", False):
            return

        # Hintergrund-Planung (Positionsspeicher) fertig? Billig (Lock+Read).
        self._poll_plan_result()

        # E-STOP / Slack: Arme drehmomentfrei/gedaempft halten und WEITER
        # senden -- ohne die Kontrolle an den Onboard-Regler zurueckzugeben
        # (der wuerde die Arme in seine Default-Pose reissen). Laeuft, bis
        # ENABLE MANIPULATION den Slack-Latch aufhebt.
        if self._arm_slack and not self.arms_enabled:
            self._publish_arm_slack()
            return

        if self.use_robot:
            robot_data = self.lowstate_subscriber.Read()
            if robot_data is not None:
                self.lowstate_buffer.SetData(robot_data)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = robot_data.motor_state[i].q
                    self.motor_state[i].dq = robot_data.motor_state[i].dq

        if not self.arms_enabled:
            if self._weight_state == "ramp_down":
                # Weiche Uebergabe: Gewicht 1->0 fahren, solange weiter senden.
                self._publish_weight_ramp_down()
            # Sonst: STILL sein. (Frueher wurden hier Beine+Taille mit kp=300
            # in self.msg geschrieben -- die gehoeren nicht in rt/arm_sdk.)
            return

        if self.walk_mode:
            # WALK: Arme auf die Lauf-Pose halten (Marker ignoriert). Die
            # Geschwindigkeitsbegrenzung unten faehrt sie sanft dorthin (kein Teleport).
            q_target = np.concatenate((self.walk_left, self.walk_right))
            # Sobald die Arme die Lauf-Pose erreicht haben: einmalig walk_ready melden,
            # damit loco_sim erst dann mit dem Laufen beginnt (Arme aufgeraeumt).
            if (not self._walk_ready_sent
                    and np.linalg.norm(q_target - self._last_q_target) < self.walk_ready_tolerance):
                self._walk_ready_sent = True
                self.walk_ready_publisher.publish(Bool(data=True))
                self.get_logger().info("Lauf-Pose erreicht -> walk_ready (Arme aufgeraeumt).")

        elif self.homing_active:
            q_target = np.concatenate((self.home_left, self.home_right))
            if np.linalg.norm(q_target - self._last_q_target) < self.homing_tolerance:

                self.homing_active = False
                self.homing_reached = True
                self._last_q_target = q_target.copy()

                if hasattr(self.ik_solver, "clear_goals"):
                    self.ik_solver.clear_goals()

                self.ik_solver.set_current_configuration({
                    "left":  self.home_left.copy(),
                    "right": self.home_right.copy()
                })

                try:
                    q_full = pin.neutral(self.ik_solver.model)
                    for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_left[i]
                    for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_right[i]

                    pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
                    pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)

                    T_left  = self.ik_solver.data.oMf[self.ik_solver._fid_left]
                    T_right = self.ik_solver.data.oMf[self.ik_solver._fid_right]

                    self._goal_left_filt  = T_left.copy()
                    self._goal_right_filt = T_right.copy()
                    if hasattr(self.ik_solver, "set_goal"):
                        self.ik_solver.set_goal("left",  T_left.copy())
                        self.ik_solver.set_goal("right", T_right.copy())

                    self._reset_after_home = True

                    self.get_logger().info("IK solver goals aligned with home pose.")
                except Exception as e:
                    self.get_logger().warning(f"Failed to align IK goals with home: {e}")

                self.get_logger().info("Home position reached.")

        elif self.homing_reached:
            q_target = np.concatenate((self.home_left, self.home_right))

        elif self._planned_motion_active:
            # Positionsspeicher: geplante Bahn abfahren -- die ausgewaehlten Arme
            # GLEICHZEITIG ueber EINE geteilte Wegpunktliste (7 DOF je Seite, in
            # _planned_sides-Reihenfolge konkateniert). Ein gemeinsamer Index ->
            # beide Arme ruecken erst weiter, wenn BEIDE den aktuellen Wegpunkt
            # erreicht haben (bleiben also synchron; die 14-DOF-Planung hat den
            # gemeinsamen Pfad Arm-zu-Arm-sicher geprueft). Marker-Beruehrung
            # bricht das ueber _planned_motion_active=False in *_goal_callback ab.
            q_target = self._last_q_target.copy()
            wps = self._planned_waypoints
            sides = self._planned_sides
            if not sides or wps is None:
                self._planned_motion_active = False
            else:
                idx = self._planned_wp_idx
                wp = wps[idx]
                # (spalten-slice im Wegpunkt, ziel-slice in q_target) je Seite
                slices = [(slice(7 * k, 7 * k + 7),
                           slice(0, 7) if side == "left" else slice(7, 14))
                          for k, side in enumerate(sides)]
                reached_wp = all(
                    np.linalg.norm(wp[col] - self._last_q_target[tgt])
                    < self.planned_motion_tolerance
                    for col, tgt in slices)
                if reached_wp and idx + 1 < len(wps):
                    idx += 1
                    self._planned_wp_idx = idx
                    wp = wps[idx]
                    reached_wp = False
                for col, tgt in slices:
                    q_target[tgt] = wp[col]
                if reached_wp and idx == len(wps) - 1:
                    self._planned_motion_active = False
                    self._align_ik_to_config(q_target[0:7], q_target[7:14])
                    self.get_logger().info(
                        f"Geplante Bewegung zu Pose '{self._planned_pose_name}' abgeschlossen.")

        else:
            # IK-Seed: Arm-Gelenke aus dem LETZTEN ZIEL (deterministisch), nicht
            # aus der Messung. Wuerde man aus der Messung seeden, fuehrt Mess-/
            # Sim-Rauschen zu leicht anderen IK-Loesungen -> minimal anderes
            # Kommando -> Arm zittert dauerhaft, ohne je einzurasten. Die Nicht-
            # Arm-Gelenke (Taille) bleiben aus der Messung, damit die FK-Basis
            # stimmt.
            if self.use_robot:
                current_all = self.get_current_motor_q()
                for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
                    current_all[jidx] = self._last_q_target[i]
                for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
                    current_all[jidx] = self._last_q_target[7 + i]
            else:
                current_all = self._assemble_full_from_last()

            try:
                self.ik_solver.set_current_configuration({
                    "left":  self._last_q_target[0:7].copy(),
                    "right": self._last_q_target[7:14].copy()
                })
            except Exception:
                pass

            if self._goal_left_filt is not None:
                self.ik_solver.set_goal("left", self._goal_left_filt)
            if self._goal_right_filt is not None:
                self.ik_solver.set_goal("right", self._goal_right_filt)

            q_dict = self.ik_solver.get_joint_targets(current_all)
            # Default: HALTEN. Ohne gesetztes IK-Ziel (kein Marker gezogen)
            # liefert get_joint_targets ein leeres Dict – dann muss der Arm in
            # seiner letzten Position bleiben. Frueher stand hier zeros(14), was
            # beide Arme beim Enable mit voller Geschwindigkeit in die
            # Null-Konfiguration (Gelenklimits/Selbstkollision) trieb -> Zappeln.
            q_target = self._last_q_target.copy()
            if "left" in q_dict:
                q_target[0:7] = q_dict["left"]
            if "right" in q_dict:
                q_target[7:14] = q_dict["right"]

        dt = self._compute_dt()
        max_step = self.arm_velocity_limit * dt
        dq = np.clip(q_target - self._last_q_target, -max_step, max_step)

        q_unsmoothed = self._last_q_target + dq
        q_smooth = (1.0 - self.ik_alpha) * self._last_q_target + self.ik_alpha * q_unsmoothed
        # Kartesisches Speedlimit (TCP + Ellbogen, ISO 10218-1 reduced speed):
        # begrenzt auch die Faelle, in denen ein kleiner Gelenkschritt eine
        # grosse Handbewegung erzeugt (gestreckter Arm, Schulterdrehung).
        q_smooth = self._limit_cartesian_speed(self._last_q_target, q_smooth, dt)
        # Selbstkollisions-Gate: nie in den eigenen Koerper fahren.
        q_smooth = self._apply_collision_gate(self._last_q_target, q_smooth)
        self._last_q_target = q_smooth.copy()

        if self.use_robot:
            self.msg.mode_machine = self.get_mode_machine()
            # mode_pr NICHT setzen (bleibt 0 wie im Unitree-arm_sdk-Beispiel;
            # betrifft nur die Knoechel-Interpretation, die uns nichts angeht).

            try:
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = \
                    float(self._advance_arm_weight(dt))
            except Exception:
                pass

            # Schwerkraft-Feedforward gegen den PD-Durchhang (s. Parameter).
            tau_g = self._arm_gravity_tau(q_smooth)

            wrist_vals = {m.value for m in G1_29_JointWristIndex}
            for idx, jid in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[jid].mode = 1
                self.msg.motor_cmd[jid].q   = float(q_smooth[idx])
                self.msg.motor_cmd[jid].dq  = 0.0
                self.msg.motor_cmd[jid].tau = float(tau_g[idx])
                if jid.value in wrist_vals:
                    self.msg.motor_cmd[jid].kp = self.kp_wrist
                    self.msg.motor_cmd[jid].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
        else:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
            js.position = [0.0] * len(js.name)
            for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[idx])
            for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[7 + idx])
            self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
