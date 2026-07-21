#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_planner.py — schlanker RRT-Connect-Planer im 7-DOF-Gelenkraum EINES Arms,
fuer den GEPLANTEN Positionsspeicher-Modus (siehe g1pilot/SCENE_BRIDGE.md
Abschnitt 9: "Positionsspeicher (Oberkoerper/Arm) — plan-execute").

Bewusst KEIN MoveIt/OMPL: die Kollisionspruefung nutzt exakt dieselben,
bereits vorhandenen und produktiv genutzten Pinocchio-Checks des IK-Solvers
(Selbstkollisions-Gate + Umgebungs-ACM, siehe utils/ik_solver.py), damit ein
gefundener Pfad garantiert dieselben Sicherheitsschranken erfuellt wie der
reaktive Servoing-Pfad -- kein zweiter, abweichender Kollisionsbegriff. Die
Planer-API ist bewusst schmal (q_start, q_goal, IK-Solver mit synchronisierter
Umgebung -> Wegpunktliste), damit sie sich spaeter durch einen anderen Planer
(z.B. MoveIt/OMPL) ersetzen liesse, ohne arm_controller.py anzufassen.

Nur EIN Arm pro Aufruf (7 DOF). Sollen beide Arme in eine gespeicherte Pose,
plant/faehrt arm_controller sie NACHEINANDER (siehe dortige Doku) -- ein
gemeinsamer 14-DOF-Planer waere robuster bei Arm-zu-Arm-Naehe, aber deutlich
aufwendiger; das bestehende Selbstkollisions-Gate deckt Arm-zu-Arm trotzdem ab,
weil der jeweils ANDERE Arm waehrend der Planung als FESTE Konfiguration in
die Kollisionspruefung eingeht (siehe _make_state_validity_fn).
"""
import time

import numpy as np

from g1pilot.utils.joints_names import LEFT_JOINT_INDICES_LIST, RIGHT_JOINT_INDICES_LIST


def _make_state_validity_fn(ik_solver, side, base_current_all):
    """Baut is_valid(q7)->bool: setzt q7 in die passenden 7 Eintraege von
    base_current_all (der ANDERE Arm + Beine/Taille bleiben auf dem zuletzt
    gemessenen/kommandierten Stand -- wirken damit als statisches Hindernis
    fuer die Selbstkollisionspruefung) und prueft Selbst- + Umgebungskollision
    MIT Sicherheitsmarge (hard=False) -- dieselbe Schwelle, die der Laufzeit-
    Regelkreis (arm_controller._apply_collision_gate) durchsetzt. Wuerde man
    hier nur auf harte Durchdringung pruefen, koennte ein frisch geplanter
    Schritt vom Laufzeit-Gate sofort wieder verworfen werden (Stillstand genau
    an der Marge-Grenze).

    WICHTIG (Nebenlaeufigkeit): plant() laeuft in einem Hintergrund-Thread,
    waehrend der 250-Hz-Regelkreis (arm_controller.main_loop) GLEICHZEITIG auf
    demselben ik_solver reaktiv weiterrechnet. Die vielen Checks hier holen
    sich deshalb EIGENE Scratch-Puffer (make_scratch_buffers()) statt der
    Live-Puffer des Solvers -- sonst wuerden zwei Threads denselben mutable
    Pinocchio-Zustand (pin.Data/GeometryData) gleichzeitig beschreiben."""
    scratch = ik_solver.make_scratch_buffers()
    idx_list = LEFT_JOINT_INDICES_LIST if side == "left" else RIGHT_JOINT_INDICES_LIST

    def is_valid(q7: np.ndarray) -> bool:
        full = base_current_all.copy()
        for i, jidx in enumerate(idx_list):
            full[jidx] = q7[i]
        if ik_solver.arm_command_in_collision(
                full, hard=False, data=scratch["data"], cdata=scratch["cdata_margin"]):
            return False
        if ik_solver.environment_command_in_collision(
                full, hard=False, data=scratch["env_data"]):
            return False
        return True

    return is_valid


def _segment_valid(is_valid, a, b, substep=0.05) -> bool:
    """Prueft ALLE Zwischenpunkte auf der Strecke a->b (nicht nur den
    Endpunkt) -- sonst koennte ein duennes Hindernis zwischen zwei RRT-Knoten
    unbemerkt durchrutschen."""
    dist = float(np.linalg.norm(b - a))
    n = max(1, int(np.ceil(dist / substep)))
    for k in range(1, n + 1):
        t = k / n
        if not is_valid(a + t * (b - a)):
            return False
    return True


class _Tree:
    def __init__(self, root: np.ndarray):
        self.nodes = [root]
        self.parents = [-1]

    def nearest(self, q):
        d = [float(np.linalg.norm(n - q)) for n in self.nodes]
        i = int(np.argmin(d))
        return i, self.nodes[i]

    def add(self, q, parent_idx):
        self.nodes.append(q)
        self.parents.append(parent_idx)
        return len(self.nodes) - 1

    def path_to_root(self, idx):
        out = []
        while idx != -1:
            out.append(self.nodes[idx])
            idx = self.parents[idx]
        return out  # Reihenfolge: Blatt -> Wurzel


def _extend(tree, target, is_valid, step_size, substep):
    """Ein RRT-Connect-EXTEND-Schritt: vom naechsten Baum-Knoten hoechstens
    step_size in Richtung target. Rueckgabe (status, index):
    status in {"reached","advanced","trapped"}."""
    idx_near, q_near = tree.nearest(target)
    delta = target - q_near
    dist = float(np.linalg.norm(delta))
    if dist < 1e-9:
        return "reached", idx_near
    q_new = target if dist <= step_size else q_near + delta * (step_size / dist)
    if not _segment_valid(is_valid, q_near, q_new, substep):
        return "trapped", None
    new_idx = tree.add(q_new, idx_near)
    return ("reached" if dist <= step_size else "advanced"), new_idx


def plan_arm_joint_path(ik_solver, side, base_current_all, q_start7, q_goal7,
                        joint_limits7, max_iters=1500, step_size=0.15,
                        substep=0.05, goal_bias=0.1, time_budget_s=8.0,
                        rng=None):
    """RRT-Connect im 7-DOF-Gelenkraum. Rueckgabe (waypoints, reason):
      - waypoints: Liste von np.ndarray(7) [q_start7 ... q_goal7] (inklusive
        beider Enden), oder None bei Fehlschlag.
      - reason: "direct" (Strecke war schon frei), "rrt_connect" (Pfad
        gefunden), "start_in_collision"/"goal_in_collision"/"no_path_found"
        bei Fehlschlag -- fuer Logging/GUI-Rueckmeldung.

    joint_limits7: Liste von (lo, hi) je Gelenk, gleiche Reihenfolge wie
    q_start7/q_goal7 (siehe utils/joints_names.JOINT_LIMITS_RAD)."""
    rng = rng or np.random.default_rng()
    is_valid = _make_state_validity_fn(ik_solver, side, base_current_all)

    q_start7 = np.asarray(q_start7, dtype=float)
    q_goal7 = np.asarray(q_goal7, dtype=float)

    if not is_valid(q_start7):
        return None, "start_in_collision"
    if not is_valid(q_goal7):
        return None, "goal_in_collision"
    if _segment_valid(is_valid, q_start7, q_goal7, substep):
        return [q_start7, q_goal7], "direct"

    lo = np.array([l for l, _ in joint_limits7], dtype=float)
    hi = np.array([h for _, h in joint_limits7], dtype=float)

    tree_a = _Tree(q_start7)
    tree_b = _Tree(q_goal7)
    a_is_start = True
    t0 = time.monotonic()

    for _ in range(max_iters):
        if time.monotonic() - t0 > time_budget_s:
            break
        if rng.random() < goal_bias:
            q_rand = q_goal7 if a_is_start else q_start7
        else:
            q_rand = lo + rng.random(len(lo)) * (hi - lo)

        status, idx = _extend(tree_a, q_rand, is_valid, step_size, substep)
        if status == "trapped":
            tree_a, tree_b = tree_b, tree_a
            a_is_start = not a_is_start
            continue

        q_new = tree_a.nodes[idx]
        status_b, idx_b = "trapped", None
        while True:
            status_b, idx_b = _extend(tree_b, q_new, is_valid, step_size, substep)
            if status_b != "advanced":
                break

        if status_b == "reached":
            path_a = list(reversed(tree_a.path_to_root(idx)))
            path_b = tree_b.path_to_root(idx_b)
            full = path_a + path_b
            if not a_is_start:
                full = list(reversed(full))
            return full, "rrt_connect"

        tree_a, tree_b = tree_b, tree_a
        a_is_start = not a_is_start

    return None, "no_path_found"


def shortcut_path(ik_solver, side, base_current_all, path, iterations=100,
                  substep=0.05, rng=None):
    """Zufaellige Shortcut-Glaettung: entfernt unnoetige Zwischenpunkte des
    RRT-Pfads, wenn die direkte Strecke zwischen zwei (nicht benachbarten)
    Wegpunkten kollisionsfrei ist. Rein kosmetisch/Effizienz -- Sicherheit
    bleibt unveraendert (dieselbe is_valid-Pruefung wie beim Planen)."""
    if len(path) <= 2:
        return list(path)
    rng = rng or np.random.default_rng()
    is_valid = _make_state_validity_fn(ik_solver, side, base_current_all)
    pts = list(path)
    for _ in range(iterations):
        if len(pts) <= 2:
            break
        i, j = sorted(rng.integers(0, len(pts), size=2))
        if j - i < 2:
            continue
        if _segment_valid(is_valid, pts[i], pts[j], substep):
            pts = pts[:i + 1] + pts[j:]
    return pts
