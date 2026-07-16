#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import pinocchio as pin
from pinocchio import SE3
from ament_index_python.packages import get_package_share_directory

from g1pilot.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)
from g1pilot.utils.helpers import (
    clamp, wrap_to_pi,
    mat_to_quat_wxyz, quat_wxyz_to_matrix,
    quat_slerp, yaw_from_R,
)


class G1IKSolver:
    """
    Inverse kinematics solver for Unitree G1-29 arms using Pinocchio,
    with smooth orientation control, damping, and optional collision avoidance.
    """

    def __init__(self,
                 urdf_path=None,
                 mesh_dir=None,
                 world_frame='pelvis',
                 frame_left='left_hand_point_contact',
                 frame_right='right_hand_point_contact',
                 alpha=0.2,
                 max_dq_step=0.05,
                 damping=1e-6,
                 max_iter=60,
                 tol=1e-4,
                 pos_gain=1.0,
                 ori_gain=0.8,
                 adaptive_damping=True,
                 sigma_min_thresh=0.08,
                 lambda_base=1e-6,
                 lambda_max=1e-1,
                 max_ori_step_rad=0.35,
                 goal_filter_alpha=0.25,
                 orientation_mode="full",
                 null_space_gain=0.15,
                 debug=False,
                 enable_collision_avoidance=True,
                 collision_margin=0.03):
        self.alpha = alpha
        self.max_dq_step = max_dq_step
        self.damping = damping
        self.max_iter = max_iter
        self.tol = tol
        self.pos_gain = pos_gain
        self.ori_gain = ori_gain
        self.adaptive_damping = adaptive_damping
        self.sigma_min_thresh = sigma_min_thresh
        self.lambda_base = lambda_base
        self.lambda_max = lambda_max
        self.max_ori_step_rad = max_ori_step_rad
        self.goal_filter_alpha = goal_filter_alpha
        self.orientation_mode = orientation_mode
        # Nullraum-Regularisierung: zieht die redundante (7. DOF) Armhaltung sanft zu
        # einer Ruhe-Pose, OHNE die Hand-Pose zu aendern -> der Arm faellt nicht in
        # den "gefalteten" (zum Koerper geklappten) IK-Zweig. 0 = aus.
        self.null_space_gain = null_space_gain
        self._rest_right = None
        self._rest_left = None
        self.debug = debug
        self.world_frame = world_frame
        self.frame_left = frame_left
        self.frame_right = frame_right

        # Selbstkollisions-Gate (s. _init_collision_gate / arm_command_in_collision)
        self.enable_collision_avoidance = enable_collision_avoidance
        self.collision_margin = float(collision_margin)

        # Load model
        try:
            if urdf_path is None or mesh_dir is None:
                pkg_share = get_package_share_directory('g1pilot')
                urdf_path = urdf_path or os.path.join(pkg_share, 'description_files', 'urdf', 'g1_29dof_inspire_ftp.urdf')
                mesh_dir = mesh_dir or os.path.join(pkg_share, 'description_files', 'meshes')

            # Load kinematic + collision models
            # Compatible version check
            print("Loading URDF model for IK solver...", flush=True)
            try:
                # Try to load both visual and collision geometries
                self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                    urdf_path,
                    package_dirs=[mesh_dir],
                    geometry_types=[pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]
                )
            except Exception as e:
                print(f"Failed to load collision model properly: {e}", flush=True)
                print("→ Retrying with collision-only model...", flush=True)
                self.model, self.collision_model = pin.buildModelsFromUrdf(
                    urdf_path,
                    package_dirs=[mesh_dir],
                    geometry_types=[pin.GeometryType.COLLISION]
                )
                self.visual_model = None


            self.data = pin.Data(self.model)

        except Exception as e:
            raise RuntimeError(f"Failed to load URDF: {e}")

        # Selbstkollisions-Gate aufbauen (kuratierte Paare + Convex-Hulls).
        # WICHTIG: buildModelsFromUrdf legt KEINE collisionPairs an -- ohne
        # diesen Schritt waere jede Kollisionspruefung ein No-Op (so war es
        # frueher: die "collision avoidance" hat nie etwas geprueft).
        self._gate_ready = False
        if self.enable_collision_avoidance:
            try:
                self._init_collision_gate()
            except Exception as e:
                print(f"[IK] WARN: Kollisions-Gate nicht verfuegbar: {e}", flush=True)

        # Joint mapping
        self._ros_joint_names = [JOINT_NAMES_ROS[i] for i in range(29)]
        self._name_to_q_index = {}
        self._name_to_v_index = {}
        for j in range(1, self.model.njoints):
            jnt = self.model.joints[j]
            if jnt.nq == 1:
                nm = self.model.names[j]
                if nm in self._ros_joint_names:
                    self._name_to_q_index[nm] = jnt.idx_q
                    self._name_to_v_index[nm] = jnt.idx_v

        # Frame IDs
        self._fid_right = self.model.getFrameId(frame_right)
        self._fid_left  = self.model.getFrameId(frame_left)

        # Maximale Armreichweite (Schulter -> Hand-TCP bei gestrecktem Arm, d.h.
        # q=0). Ziele werden in solve() auf diese Kugel um die Schulter geklemmt:
        # der rviz-Marker laesst sich beliebig weit ziehen, aber ein Ziel JENSEITS
        # der Reichweite darf den Solver nicht in wilde Konfigurationen treiben
        # (Ursache fuer "Arm faehrt hinter den Kopf und kommt nie zurueck").
        self._fid_shoulder = {}
        self._arm_reach = {}
        for side, sh_link, fid in (("right", "right_shoulder_pitch_link", self._fid_right),
                                   ("left",  "left_shoulder_pitch_link",  self._fid_left)):
            try:
                fid_sh = self.model.getFrameId(sh_link)
                # Reichweite NICHT bei q=0 messen (dort ist der Arm wegen des
                # TCP-Offsets nicht gestreckt), sondern ueber Ellbogen/Handgelenk
                # samplen und das Maximum nehmen.
                elb = self._name_to_q_index[f"{side}_elbow_joint"]
                wrp = self._name_to_q_index[f"{side}_wrist_pitch_joint"]
                reach = 0.0
                q0 = pin.neutral(self.model)
                for e in np.linspace(-1.0472, 2.0944, 24):
                    for w in np.linspace(-1.6144, 1.6144, 9):
                        q0[elb] = e
                        q0[wrp] = w
                        pin.forwardKinematics(self.model, self.data, q0)
                        pin.updateFramePlacements(self.model, self.data)
                        d = float(np.linalg.norm(
                            self.data.oMf[fid].translation - self.data.oMf[fid_sh].translation))
                        reach = max(reach, d)
                self._fid_shoulder[side] = fid_sh
                self._arm_reach[side] = reach
            except Exception:
                self._fid_shoulder[side] = None
                self._arm_reach[side] = None
        # Sicherheitsmarge: knapp INNERHALB der Kugel bleiben, sonst haengt der
        # Arm dauerhaft in der gestreckten Singulaeritaet.
        self.reach_margin = 0.97

        # State buffers
        self._goal_right = None
        self._goal_left  = None
        self._prev_q_full = None
        self._prev_q14 = None

    # --------------------------------------------------------
    # Helper functions
    # --------------------------------------------------------

    def _limit_ori_step(self, R_cur, R_des):
        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        norm = np.linalg.norm(aa)
        if norm < 1e-12 or norm <= self.max_ori_step_rad:
            return R_des
        aa_limited = aa * (self.max_ori_step_rad / norm)
        return R_cur @ pin.exp3(aa_limited)

    def _lowpass_goal(self, T_prev, T_new):
        if T_prev is None:
            return T_new
        p = (1 - self.goal_filter_alpha) * T_prev.translation + self.goal_filter_alpha * T_new.translation
        q0 = mat_to_quat_wxyz(T_prev.rotation)
        q1 = mat_to_quat_wxyz(T_new.rotation)
        qf = quat_slerp(q0, q1, self.goal_filter_alpha)
        return SE3(quat_wxyz_to_matrix(qf), p)

    # --------------------------------------------------------
    # Selbstkollisions-Gate
    # --------------------------------------------------------
    #
    # Konzept: KEINE Distanz-Abfragen (computeDistances auf den G1-Meshes
    # kostet ~2 s pro Aufruf -> unbrauchbar), sondern ein BINAERER
    # Kollisionscheck mit security_margin auf kuratierten Paaren und
    # Convex-Hull-Geometrien. Gemessen: ~0.3 ms pro Check -> laeuft in
    # jedem 250-Hz-Tick des arm_controller. Der Controller prueft damit die
    # KOMMANDIERTE Konfiguration, BEVOR sie zum Roboter geht, und haelt die
    # Arme an, statt in den eigenen Koerper zu fahren.

    # Bewegliche Armteile, die den Koerper treffen koennen. shoulder_pitch/
    # shoulder_roll bewegen sich kaum vom Torso weg; shoulder_yaw sitzt
    # konstruktionsbedingt dauerhaft ~2.8 cm am Torso und wuerde mit Marge
    # permanent "kollidieren" -> bewusst NICHT gegen den Torso geprueft.
    _GATE_ARM_KEYS = ("elbow", "wrist", "base_link", "palm")
    _GATE_BODY_GEOMS = (
        "torso_link_0", "head_link_0", "pelvis_contour_link_0", "logo_link_0",
        "left_hip_pitch_link_0", "right_hip_pitch_link_0",
        "left_hip_roll_link_0", "right_hip_roll_link_0",
        "left_hip_yaw_link_0", "right_hip_yaw_link_0",
        "left_knee_link_0", "right_knee_link_0",
    )

    def _init_collision_gate(self):
        cm = self.collision_model
        # 1) Meshes -> Convex-Hulls: schneller UND konservativer (Hull
        #    umschliesst das Mesh). Nicht jede hppfcl/coal-Version kann das ->
        #    per hasattr abgesichert; ohne Hulls funktioniert der Check auch,
        #    nur langsamer.
        for g in cm.geometryObjects:
            geom = g.geometry
            if hasattr(geom, "buildConvexRepresentation"):
                try:
                    geom.buildConvexRepresentation(False)
                    if geom.convex is not None:
                        g.geometry = geom.convex
                except Exception:
                    pass

        # 2) Kuratierte Paare: bewegliche Armteile gegen Koerper + Arm gegen Arm.
        names = {g.name for g in cm.geometryObjects}
        moving = {
            side: [n for n in names
                   if n.startswith(side) and any(k in n for k in self._GATE_ARM_KEYS)
                   # Finger-Geometrien nicht pruefen: die Hand darf greifen;
                   # die Handbasis (base_link/palm) deckt den Koerperschutz ab.
                   and not any(f in n for f in ("index", "middle", "ring",
                                                "little", "thumb"))]
            for side in ("left", "right")
        }
        body = [n for n in self._GATE_BODY_GEOMS if n in names]
        for side in ("left", "right"):
            for a in moving[side]:
                for b in body:
                    cm.addCollisionPair(pin.CollisionPair(
                        cm.getGeometryId(a), cm.getGeometryId(b)))
        for a in moving["left"]:
            for b in moving["right"]:
                cm.addCollisionPair(pin.CollisionPair(
                    cm.getGeometryId(a), cm.getGeometryId(b)))

        # 3) Zwei GeometryData: mit Sicherheitsmarge (Fruehwarnband) und hart
        #    (echte Durchdringung). Eigenes pin.Data, damit der Gate-FK nicht
        #    self.data (Solver-Zustand) ueberschreibt.
        self._gate_data = pin.Data(self.model)
        self._gate_cdata_margin = pin.GeometryData(cm)
        self._gate_cdata_hard = pin.GeometryData(cm)
        try:
            for req in self._gate_cdata_margin.collisionRequests:
                req.security_margin = self.collision_margin
        except Exception:
            # Alte hppfcl ohne security_margin: Marge entfaellt, harter
            # Check bleibt wirksam.
            self._gate_cdata_margin = self._gate_cdata_hard
        self._gate_ready = True
        print(f"[IK] Kollisions-Gate aktiv: {len(cm.collisionPairs)} Paare, "
              f"Marge {self.collision_margin*100:.0f} cm.", flush=True)

    def arm_command_in_collision(self, current_all, hard=False) -> bool:
        """True, wenn die 29-DOF-Konfiguration current_all (ROS-Gelenkreihenfolge)
        eine Selbstkollision (hard=True) bzw. eine Annaeherung unter die
        Sicherheitsmarge (hard=False) enthaelt. Bei nicht verfuegbarem Gate
        immer False (Verhalten wie bisher, aber der Controller loggt das)."""
        if not self._gate_ready:
            return False
        q = pin.neutral(self.model)
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(current_all[jid_idx])
        cdata = self._gate_cdata_hard if hard else self._gate_cdata_margin
        return bool(pin.computeCollisions(
            self.model, self._gate_data, self.collision_model, cdata, q, True))



    # --------------------------------------------------------
    # Public API
    # --------------------------------------------------------

    def set_current_configuration(self, q_dict: dict):
        """
        Manually set the current joint configuration for the solver.
        This is useful to reset the internal state (e.g., after homing).

        Parameters
        ----------
        q_dict : dict
            Dictionary with optional keys 'left' and/or 'right', each
            containing a 7-element numpy array of joint angles in radians.
        """
        q_full = pin.neutral(self.model)

        if 'left' in q_dict:
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                q_full[self._name_to_q_index[self._ros_joint_names[arm_i]]] = q_dict['left'][i]

        if 'right' in q_dict:
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                q_full[self._name_to_q_index[self._ros_joint_names[arm_i]]] = q_dict['right'][i]

        self._prev_q_full = q_full.copy()
        self._prev_q14 = np.concatenate([
            q_dict.get('left', np.zeros(7)),
            q_dict.get('right', np.zeros(7))
        ])


    def set_rest_posture(self, side, q7):
        """Ruhe-Pose (7 DOF) fuer die Nullraum-Regularisierung setzen. Der redundante
        Freiheitsgrad wird sanft hierhin gezogen, ohne die Hand-Pose zu beeinflussen."""
        arr = np.array(q7, dtype=float).copy()
        if side == "right":
            self._rest_right = arr
        elif side == "left":
            self._rest_left = arr

    def set_goal(self, side, T_goal: SE3):
        """Set a new SE3 target for the given side ('left' or 'right')."""
        if side == "right":
            self._goal_right = self._lowpass_goal(self._goal_right, T_goal)
        elif side == "left":
            self._goal_left = self._lowpass_goal(self._goal_left, T_goal)

    def clear_goals(self):
        """Beide IK-Ziele verwerfen (z.B. beim Homing). Wurde vom arm_controller
        schon immer via hasattr aufgerufen, existierte aber nicht -> alte Ziele
        blieben ueber das Homing hinaus aktiv."""
        self._goal_right = None
        self._goal_left = None

    def get_joint_targets(self, current_all: np.ndarray, q_init: np.ndarray = None):
        """
        Compute current joint targets (left/right arms) given current joint positions.
        Returns a dictionary with 'left' and/or 'right' arrays (7 DOF each).
        """
        out = {}
        if self._goal_left is not None:
            q_left = self.solve('left', q_init, current_all)
            if q_left is not None:
                out['left'] = q_left
        if self._goal_right is not None:
            q_right = self.solve('right', q_init, current_all)
            if q_right is not None:
                out['right'] = q_right
        return out

    def _clamp_goal_to_reach(self, side, goal, q):
        """Ziel-Translation auf die Reichweiten-Kugel um die Schulter klemmen.
        Unerreichbare Ziele (Marker zu weit gezogen) wuerden den DLS-Solver sonst
        durch die gestreckte Singulaeritaet in den gefalteten/ueber-Kopf-Zweig
        druecken. Geklemmt zeigt der Arm stattdessen sauber Richtung Ziel."""
        fid_sh = self._fid_shoulder.get(side)
        reach = self._arm_reach.get(side)
        if fid_sh is None or reach is None:
            return goal
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        p_sh = self.data.oMf[fid_sh].translation
        v = goal.translation - p_sh
        d = float(np.linalg.norm(v))
        r_max = self.reach_margin * reach
        if d <= r_max or d < 1e-9:
            return goal
        return SE3(goal.rotation.copy(), p_sh + v * (r_max / d))

    def solve(self, side: str, q_init: np.ndarray, current_all: np.ndarray) -> np.ndarray:
        """Compute 7-DOF IK solution for one arm (collision-aware)."""
        fid = self._fid_right if side == 'right' else self._fid_left
        arm_ids = RIGHT_JOINT_INDICES_LIST if side == 'right' else LEFT_JOINT_INDICES_LIST
        goal = self._goal_right if side == 'right' else self._goal_left
        if goal is None:
            return None

        # Initialize joint vector
        q = q_init.copy() if q_init is not None else pin.neutral(self.model)
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(current_all[jid_idx])

        # Unerreichbare Ziele auf die Reichweiten-Kugel klemmen (s.o.).
        goal = self._clamp_goal_to_reach(side, goal, q)

        # Startfehler merken: falls die Iteration das Ziel VERSCHLECHTERT
        # (Divergenz, z.B. nahe Singulaeritaet), geben wir unten die
        # Ausgangskonfiguration zurueck statt einer wilden Pose.
        q_seed_arm = np.array(
            [q[self._name_to_q_index[self._ros_joint_names[i]]] for i in arm_ids])
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        err0 = np.linalg.norm(pin.log(self.data.oMf[fid].inverse() * goal).vector)

        # Iterative IK loop
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            M_cur = self.data.oMf[fid]
            R_des = self._limit_ori_step(M_cur.rotation, goal.rotation)
            T_des = SE3(R_des, goal.translation)

            # Frame Jacobian. WICHTIG: pin.LOCAL, weil der Pose-Fehler unten ein
            # LOCAL-Twist ist (pin.log der Relativtransformation). Frueher stand
            # hier LOCAL_WORLD_ALIGNED -> Jacobian und Fehler waren um die
            # aktuelle Handrotation gegeneinander verdreht -> der Solver schob
            # den Arm systematisch in die falsche Richtung (nach oben/hinter den
            # Kopf statt zum Marker) und blieb dort haengen.
            J6 = pin.computeFrameJacobian(self.model, self.data, q, fid, pin.LOCAL)
            J_eff = J6[:, [self._name_to_v_index[self._ros_joint_names[i]] for i in arm_ids]]

            # Pose error (LOCAL-Twist, konsistent zur LOCAL-Jacobian)
            err6 = pin.log(M_cur.inverse() * T_des).vector

            if np.linalg.norm(err6) < self.tol:
                break

            # Trust-Region: Fehler pro Iteration begrenzen, damit der DLS-Schritt
            # im linearen Bereich bleibt. Bei weit gezogenem Marker (0.3+ m) wuerde
            # J^+ * err sonst riesige dq liefern, die das Clipping unten richtungs-
            # verfaelscht -> der Arm laeuft schraeg weg statt zum Ziel.
            p_norm = float(np.linalg.norm(err6[:3]))
            if p_norm > 0.10:
                err6[:3] *= 0.10 / p_norm
            err6[:3] *= self.pos_gain
            err6[3:] *= self.ori_gain

            def _damping(J):
                lam = self.lambda_base
                if self.adaptive_damping:
                    svals = np.linalg.svd(J, compute_uv=False)
                    sigma_min = float(np.min(svals)) if len(svals) > 0 else 0.0
                    if sigma_min < self.sigma_min_thresh:
                        frac = clamp((self.sigma_min_thresh - sigma_min) / self.sigma_min_thresh, 0, 1)
                        lam = self.lambda_base + frac * (self.lambda_max - self.lambda_base)
                return lam

            def _dpinv(J):
                lam = _damping(J)
                return J.T @ np.linalg.inv(J @ J.T + lam * np.eye(J.shape[0]))

            # TASK-PRIORITAET statt gemischtem 6D-DLS: Position ist Primaeraufgabe,
            # Orientierung laeuft im Nullraum der Position. An der Reichweiten-
            # grenze (Marker weit gezogen) opferte der gemischte Ansatz sonst
            # die POSITION fuer eine perfekte Orientierung -> Hand blieb weit vor
            # dem Marker stehen / verrenkte sich. So faehrt die Hand immer so nah
            # wie moeglich ans Ziel und nutzt den Rest fuer die Orientierung.
            J_p, J_o = J_eff[:3, :], J_eff[3:, :]
            e_p, e_o = err6[:3], err6[3:]
            Jp_pinv = _dpinv(J_p)
            dq_red = Jp_pinv @ e_p
            N_p = np.eye(len(arm_ids)) - Jp_pinv @ J_p
            J_on = J_o @ N_p
            dq_red = dq_red + _dpinv(J_on) @ (e_o - J_o @ dq_red)

            # Nullraum-Haltungs-Regularisierung: zieht die redundante Armhaltung zur
            # Ruhe-Pose, PROJIZIERT in den Nullraum der Hand-Aufgabe (N = I - J^+ J) ->
            # die Hand-Pose bleibt unveraendert, aber der Arm faellt nicht in den zum
            # Koerper gefalteten Zweig (Ursache fuer das "faehrt in falsche Richtung").
            rest = self._rest_right if side == 'right' else self._rest_left
            if rest is not None and self.null_space_gain > 0.0:
                q_arm = np.array(
                    [q[self._name_to_q_index[self._ros_joint_names[i]]] for i in arm_ids])
                J_pinv = _dpinv(J_eff)
                N = np.eye(len(arm_ids)) - J_pinv @ J_eff
                dq_red += N @ (self.null_space_gain * (rest - q_arm))

            # Integrate step. Schritt RICHTUNGSERHALTEND skalieren (statt per
            # Gelenk zu clippen): per-Gelenk-Clipping verdreht die Schrittrichtung,
            # sobald mehrere Gelenke im Limit sind -> der Arm driftet seitlich weg.
            max_abs = float(np.max(np.abs(dq_red))) if len(dq_red) else 0.0
            if max_abs > self.max_dq_step:
                dq_red = dq_red * (self.max_dq_step / max_abs)
            for i, arm_i in enumerate(arm_ids):
                qi = self._name_to_q_index[self._ros_joint_names[arm_i]]
                q[qi] = np.clip(q[qi] + dq_red[i], *JOINT_LIMITS_RAD[arm_i])

        # Divergenz-Schutz: hat die Iteration den Pose-Fehler VERGROESSERT
        # (Singulaeritaet/Grenzlage), lieber die Ausgangskonfiguration halten,
        # statt eine entgleiste Loesung zu kommandieren.
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        err1 = np.linalg.norm(pin.log(self.data.oMf[fid].inverse() * goal).vector)
        if err1 > err0 + 1e-6:
            if self.debug:
                print(f"[IK] {side}: divergiert (err {err0:.4f} -> {err1:.4f}), halte Pose.",
                      flush=True)
            return q_seed_arm

        # Return arm subset (7 joints)
        return np.array([float(q[self._name_to_q_index[self._ros_joint_names[i]]]) for i in arm_ids], dtype=float)

