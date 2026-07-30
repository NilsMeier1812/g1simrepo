#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_planner.py — Wegplaner im 7-DOF-Gelenkraum EINES Arms, fuer den GEPLANTEN
Positionsspeicher-Modus (siehe g1pilot/SCENE_BRIDGE.md Abschnitt 9:
"Positionsspeicher (Oberkoerper/Arm) — plan-execute").

PLANER-BACKEND: OMPL (RRTConnect) mit einem SCHLANKEN, HANDGESCHRIEBENEN
RRT-Connect als Fallback, falls die OMPL-Python-Bindings im Container nicht
verfuegbar sind. Beide Backends nutzen EXAKT dieselbe Kollisionspruefung: die
bereits vorhandenen und produktiv genutzten Pinocchio-Checks des IK-Solvers
(Selbstkollisions-Gate + Umgebungs-ACM, siehe utils/ik_solver.py). Ein
gefundener Pfad erfuellt damit garantiert dieselben Sicherheitsschranken wie
der reaktive Servoing-Pfad -- kein zweiter, abweichender Kollisionsbegriff.

WARUM OMPL statt weiter handgeschrieben? OMPL bringt ausgereifte, breit
getestete Planer (RRTConnect, RRT*, BIT*, ...) und eine robuste
Pfad-Vereinfachung mit. Die Planer-API hier bleibt aber bewusst schmal
(q_start, q_goal, IK-Solver mit synchronisierter Umgebung -> Wegpunktliste),
sodass arm_controller.py NICHT angefasst werden muss (der Rueckgabe-Kontrakt
ist identisch geblieben).

SICHERHEIT / DISKRETISIERUNG: OMPLs Motion-Validator prueft eine Strecke in
festen Schritten ab, deren Aufloesung als BRUCHTEIL der Raum-Ausdehnung
angegeben wird. Zu grob -> duenne Hindernisse werden uebersprungen. Wir koppeln
die Aufloesung deshalb an denselben absoluten Schritt (`substep`, in rad), den
auch der Fallback nutzt: resolution = substep / raum_ausdehnung. Zusaetzlich
wird JEDER von OMPL gelieferte Pfad danach nochmals mit exakt derselben
Segment-Pruefung (_segment_valid) wie der Fallback nachvalidiert -- schlaegt
das fehl (etwa durch einen Diskretisierungs-Rest), faellt der Aufruf auf den
handgeschriebenen Planer zurueck.

EIN ODER BEIDE Arme pro Aufruf (7 bzw. 14 DOF, siehe plan_arms_joint_path):
Sollen beide Arme GLEICHZEITIG in eine gespeicherte Pose, plant arm_controller
sie GEMEINSAM als 14-DOF-Problem -- damit deckt der Selbstkollisions-Check
Arm-gegen-Arm an JEDEM Zwischenzustand ab, und beide Arme koennen synchron
ueber EINE geteilte Wegpunktliste abgefahren werden. Wird nur ein Arm geplant
(7 DOF), gilt der andere Arm als feste Konfiguration (statisches Hindernis) --
er wird nicht mitbewegt.
"""
import math
import time

import numpy as np

from g1pilot.utils.joints_names import LEFT_JOINT_INDICES_LIST, RIGHT_JOINT_INDICES_LIST

# ── OMPL-Bindings optional laden ─────────────────────────────────────────
# Fehlen sie (schlankes Image ohne ompl-Wheel), faellt plan_arm_joint_path
# transparent auf den handgeschriebenen RRT-Connect zurueck -- der
# Positionsspeicher funktioniert also auch ohne OMPL, nur mit dem simpleren
# Planer. So bleibt das Feature nicht an einer optionalen Abhaengigkeit haengen.
try:
    from ompl import base as _ob
    from ompl import geometric as _og
    from ompl import util as _ou
    _ou.setLogLevel(_ou.LOG_WARN)   # OMPLs Info-Spam aus stderr heraushalten
    OMPL_AVAILABLE = True
except Exception:   # ImportError, aber auch evtl. Laufzeit-/ABI-Fehler beim Laden
    OMPL_AVAILABLE = False


def _sides_list(sides):
    """Normalisiert die Seiten-Angabe: einzelner String -> Liste. Erlaubte
    Werte 'left'/'right'. Reihenfolge = Spaltenreihenfolge im Gelenkvektor."""
    if isinstance(sides, str):
        return [sides]
    return list(sides)


def _joint_indices_for(sides):
    """Konkateniert die 7 Arm-Gelenkindizes je Seite in der gegebenen
    Reihenfolge -> Gesamt-Indexliste (7 bzw. 14 Eintraege)."""
    idx = []
    for s in _sides_list(sides):
        idx += list(LEFT_JOINT_INDICES_LIST if s == "left" else RIGHT_JOINT_INDICES_LIST)
    return idx


def _make_state_validity_fn(ik_solver, sides, base_current_all):
    """Baut is_valid(qN)->bool fuer EINEN oder BEIDE Arme (N = 7*Anzahl Seiten):
    setzt qN in die passenden Eintraege von base_current_all (die NICHT
    geplanten Gelenke -- ggf. der andere Arm + Beine/Taille -- bleiben auf dem
    zuletzt gemessenen/kommandierten Stand, wirken also als statisches
    Hindernis) und prueft Selbst- + Umgebungskollision MIT Sicherheitsmarge
    (hard=False) -- dieselbe Schwelle, die der Laufzeit-Regelkreis
    (arm_controller._apply_collision_gate) durchsetzt.

    Werden BEIDE Arme uebergeben (14 DOF), deckt genau derselbe
    Selbstkollisions-Check auch Arm-gegen-Arm an JEDEM Zwischenzustand ab --
    das ist der Grund, warum die gleichzeitige (14-DOF-)Planung sicher ist,
    waehrend eine getrennte 7+7-Planung mit simultaner Ausfuehrung die
    Arm-zu-Arm-Naehe waehrend des Transits NICHT pruefen wuerde.

    WICHTIG (Nebenlaeufigkeit): plant() laeuft in einem Hintergrund-Thread,
    waehrend der 250-Hz-Regelkreis (arm_controller.main_loop) GLEICHZEITIG auf
    demselben ik_solver reaktiv weiterrechnet. Die vielen Checks hier holen
    sich deshalb EIGENE Scratch-Puffer (make_scratch_buffers()) statt der
    Live-Puffer des Solvers -- sonst wuerden zwei Threads denselben mutable
    Pinocchio-Zustand (pin.Data/GeometryData) gleichzeitig beschreiben. Gilt
    auch fuer den OMPL-Pfad: OMPLs StateValidityChecker ruft dieselbe is_valid
    (und damit dieselben Scratch-Puffer) aus dem Planungs-Thread heraus auf --
    RRTConnect plant single-threaded, daher genuegt EIN Puffersatz pro Aufruf."""
    scratch = ik_solver.make_scratch_buffers()
    idx_list = _joint_indices_for(sides)

    def is_valid(qN: np.ndarray) -> bool:
        full = base_current_all.copy()
        for i, jidx in enumerate(idx_list):
            full[jidx] = qN[i]
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
    Endpunkt) -- sonst koennte ein duennes Hindernis zwischen zwei Knoten
    unbemerkt durchrutschen."""
    dist = float(np.linalg.norm(b - a))
    n = max(1, int(np.ceil(dist / substep)))
    for k in range(1, n + 1):
        t = k / n
        if not is_valid(a + t * (b - a)):
            return False
    return True


def _path_segments_valid(is_valid, waypoints, substep=0.05) -> bool:
    """Nachvalidierung: prueft JEDES Segment einer fertigen Wegpunktliste mit
    exakt derselben Segment-Pruefung wie der Fallback-Planer. Damit haengt die
    Sicherheit NICHT an OMPLs interner Diskretisierung -- der zurueckgegebene
    Pfad genuegt garantiert demselben Kollisionsbegriff wie ein Fallback-Pfad."""
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        if not _segment_valid(is_valid, np.asarray(a, dtype=float),
                              np.asarray(b, dtype=float), substep):
            return False
    return True


# ══════════════════════════════════════════════════════════════════════
# OMPL-Backend (Primaer)
# ══════════════════════════════════════════════════════════════════════
def _plan_ompl(is_valid, q_start7, q_goal7, joint_limits7,
               step_size, substep, time_budget_s):
    """Plant mit OMPL RRTConnect im 7-DOF-RealVectorStateSpace. Rueckgabe:
    Liste von np.ndarray(7) (inkl. beider Enden) oder None. Start/Ziel/Direkt
    wurden vom Aufrufer bereits geprueft; hier geht es nur um den Umweg."""
    dim = len(q_start7)
    lo = [float(l) for l, _ in joint_limits7]
    hi = [float(h) for _, h in joint_limits7]

    space = _ob.RealVectorStateSpace(dim)
    bounds = _ob.RealVectorBounds(dim)
    for i in range(dim):
        # Start/Ziel koennen minimal ausserhalb der nominellen Limits liegen
        # (Messrauschen); Grenzen leicht weiten, damit sie im Raum liegen.
        lb = min(lo[i], float(q_start7[i]), float(q_goal7[i]))
        ub = max(hi[i], float(q_start7[i]), float(q_goal7[i]))
        bounds.setLow(i, lb)
        bounds.setHigh(i, ub)
    space.setBounds(bounds)

    setup = _og.SimpleSetup(space)
    si = setup.getSpaceInformation()

    class _Checker(_ob.StateValidityChecker):
        def __init__(self, si_):
            super().__init__(si_)

        def isValid(self, state):
            q = np.fromiter((state[i] for i in range(dim)), dtype=float, count=dim)
            return bool(is_valid(q))

    checker = _Checker(si)
    setup.setStateValidityChecker(checker)

    # Aufloesung an den absoluten substep (rad) koppeln (siehe Modul-Doku):
    # OMPL erwartet einen Bruchteil der maximalen Raum-Ausdehnung.
    extent = math.sqrt(sum((hi[i] - lo[i]) ** 2 for i in range(dim))) or 1.0
    si.setStateValidityCheckingResolution(min(0.05, substep / extent))

    start = space.allocState()
    goal = space.allocState()
    for i in range(dim):
        start[i] = float(q_start7[i])
        goal[i] = float(q_goal7[i])
    setup.setStartAndGoalStates(start, goal)

    planner = _og.RRTConnect(si)
    planner.setRange(float(step_size))
    setup.setPlanner(planner)

    # Kein RNG-Seeding: OMPL nutzt eine GLOBALE, bereits beim Konstruieren des
    # Planers gestartete RNG -- ein spaeteres setSeed waere wirkungslos und
    # wuerde nur "RNG already started" auf stderr spammen. Zeitbasierte
    # Default-Seed = nicht-deterministisch pro Aufruf, genau wie der Fallback
    # (der Controller ruft ohne festes rng auf -> frische Zufalls-Seed).

    setup.setup()
    setup.solve(float(time_budget_s))
    if not setup.haveExactSolutionPath():
        return None   # nur eine Naeherung -> als Fehlschlag behandeln

    setup.simplifySolution(min(2.0, time_budget_s * 0.25))
    path = setup.getSolutionPath()

    # In dichtere Wegpunkte aufloesen (Segmentlaenge ~substep), damit der
    # nachgelagerte Rate-Limiter des Controllers nah an der geprueften Geraden
    # bleibt und die Nachvalidierung feine Stuetzstellen hat.
    length = float(path.length())
    n = max(2, int(math.ceil(length / substep)) + 1)
    path.interpolate(n)

    count = path.getStateCount()
    waypoints = []
    for k in range(count):
        s = path.getState(k)
        waypoints.append(np.fromiter((s[i] for i in range(dim)), dtype=float, count=dim))
    return waypoints


# ══════════════════════════════════════════════════════════════════════
# Fallback-Backend: handgeschriebener RRT-Connect (ohne OMPL)
# ══════════════════════════════════════════════════════════════════════
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


def _plan_rrt_connect_builtin(is_valid, q_start7, q_goal7, joint_limits7,
                              max_iters, step_size, substep, goal_bias,
                              time_budget_s, rng):
    """Handgeschriebener RRT-Connect. Erwartet, dass Start/Ziel gueltig und die
    direkte Strecke NICHT frei ist (der Aufrufer hat das bereits geprueft).
    Rueckgabe: Wegpunktliste oder None."""
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
            return full

        tree_a, tree_b = tree_b, tree_a
        a_is_start = not a_is_start

    return None


# ══════════════════════════════════════════════════════════════════════
# Oeffentliche API (Rueckgabe-Kontrakt unveraendert)
# ══════════════════════════════════════════════════════════════════════
def plan_arms_joint_path(ik_solver, sides, base_current_all, q_start, q_goal,
                         joint_limits, max_iters=1500, step_size=0.15,
                         substep=0.05, goal_bias=0.1, time_budget_s=8.0,
                         rng=None):
    """Plant einen kollisionsfreien Gelenkraum-Pfad fuer EINEN oder BEIDE Arme.

    `sides`: 'left'/'right' oder Liste davon (z.B. ['left','right'] fuer eine
    gemeinsame 14-DOF-Planung -> gleichzeitige, Arm-zu-Arm-sichere Bewegung).
    q_start/q_goal/joint_limits sind ueber die Seiten in genau dieser
    Reihenfolge konkateniert (7 Eintraege je Seite).

    Rueckgabe (waypoints, reason):
      - waypoints: Liste von np.ndarray(7*Seiten) [q_start ... q_goal] (inkl.
        beider Enden), oder None bei Fehlschlag.
      - reason: "direct" | "ompl_rrtconnect" | "rrt_connect" |
        "start_in_collision" | "goal_in_collision" | "no_path_found"."""
    rng = rng or np.random.default_rng()
    is_valid = _make_state_validity_fn(ik_solver, sides, base_current_all)

    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)

    # Schnelle, deterministische Vorpruefungen -- identischer Sicherheitsbegriff
    # fuer BEIDE Backends, mit klaren Reason-Codes.
    if not is_valid(q_start):
        return None, "start_in_collision"
    if not is_valid(q_goal):
        return None, "goal_in_collision"
    if _segment_valid(is_valid, q_start, q_goal, substep):
        return [q_start, q_goal], "direct"

    # Primaer: OMPL. Der gelieferte Pfad wird mit derselben Segment-Pruefung
    # wie der Fallback nachvalidiert; schlaegt das fehl, wird der Fallback
    # verwendet (Sicherheit haengt NICHT an OMPLs interner Diskretisierung).
    if OMPL_AVAILABLE:
        try:
            wps = _plan_ompl(is_valid, q_start, q_goal, joint_limits,
                             step_size, substep, time_budget_s)
        except Exception:
            wps = None
        if wps is not None and _path_segments_valid(is_valid, wps, substep):
            return wps, "ompl_rrtconnect"
        # OMPL fand nichts (Gueltiges) -> Fallback versuchen.

    path = _plan_rrt_connect_builtin(
        is_valid, q_start, q_goal, joint_limits,
        max_iters, step_size, substep, goal_bias, time_budget_s, rng)
    if path is None:
        return None, "no_path_found"
    return path, "rrt_connect"


def plan_arm_joint_path(ik_solver, side, base_current_all, q_start7, q_goal7,
                        joint_limits7, **kwargs):
    """Rueckwaerts-kompatibler Einzel-Arm-Wrapper um plan_arms_joint_path
    (7 DOF, EIN Arm). Neuer Code sollte direkt plan_arms_joint_path nutzen."""
    return plan_arms_joint_path(ik_solver, [side], base_current_all,
                                q_start7, q_goal7, joint_limits7, **kwargs)


def shortcut_path(ik_solver, sides, base_current_all, path, iterations=100,
                  substep=0.05, rng=None):
    """Zufaellige Shortcut-Glaettung: entfernt unnoetige Zwischenpunkte des
    Pfads, wenn die direkte Strecke zwischen zwei (nicht benachbarten)
    Wegpunkten kollisionsfrei ist. Backend-unabhaengig -- funktioniert auf
    jeder Wegpunktliste (OMPL wie Fallback, 7 wie 14 DOF) und nutzt exakt
    dieselbe is_valid-Pruefung wie beim Planen. `sides` wie bei
    plan_arms_joint_path. Rein kosmetisch/Effizienz -- Sicherheit unveraendert."""
    if len(path) <= 2:
        return list(path)
    rng = rng or np.random.default_rng()
    is_valid = _make_state_validity_fn(ik_solver, sides, base_current_all)
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
