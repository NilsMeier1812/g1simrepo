#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_hold_test — REINER PHYSIK-Test der Arm-Stabilitaet, OHNE DDS/ROS/arm_controller.

Laedt dieselbe MuJoCo-Szene wie unitree_mujoco.py, richtet HOLD_BASE genauso ein
(Weld/Steifigkeit/Teleport via --mode) und haelt ALLE Armgelenke mit einem
konstanten In-Process-PD (gleiche Formel wie die Bridge: tau=kp*(q*-q)+kd*(0-dq))
auf einer schwerkraftbelasteten Zielpose. Es misst, wie stark die GEMESSENEN
Armgelenke um die konstante Sollpose schwingen.

Damit laesst sich das "Zittern" eindeutig verorten:
  * Zittern HIER (kein DDS, konstanter 200-Hz-PD)  -> reine MuJoCo-Physik:
        HOLD_BASE-Modus / Gains / Integration. -> --mode weld vs teleport vs off
        und --kp/--kd vergleichen.
  * HIER ruhig, aber ueber den arm_controller zittrig -> Problem liegt in der
        DDS-/Bridge-/Controller-Schicht (Publish-Rate, Smoothing, ...).

Aufruf (im MuJoCo-Container, headless):
    cd /unitree_mujoco/simulate_python
    python3 sim_hold_test.py --mode weld     --seconds 8
    python3 sim_hold_test.py --mode teleport --seconds 8
    python3 sim_hold_test.py --mode weld --kd 20 --seconds 8
    python3 sim_hold_test.py --mode weld --view        # mit Viewer zuschauen
"""
import argparse
import time
import types

import numpy as np
import mujoco
import mujoco.viewer  # noqa: F401  (nur fuer --view genutzt)

import config as sim_config
from hold_base import HoldBase

# Arm-Motorindizes (= ctrl-/sensor-Index). left 15..21, right 22..28.
LEFT_ARM = [15, 16, 17, 18, 19, 20, 21]
RIGHT_ARM = [22, 23, 24, 25, 26, 27, 28]
ARM = LEFT_ARM + RIGHT_ARM

# Schwerkraftbelastete Zielpose je Arm:
#  [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll, wrist_pitch, wrist_yaw]
TARGET_LEFT = [0.3,  0.2, 0.0, 1.0, 0.0, 0.0, 0.0]
TARGET_RIGHT = [0.3, -0.2, 0.0, 1.0, 0.0, 0.0, 0.0]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", default="weld", choices=["weld", "teleport", "off"],
                   help="HOLD_BASE-Modus fuer diesen Test")
    p.add_argument("--kp", type=float, default=150.0)
    p.add_argument("--kd", type=float, default=12.0)
    p.add_argument("--seconds", type=float, default=8.0, help="Sim-Zeit")
    p.add_argument("--settle", type=float, default=1.0, help="Einschwingzeit vor Messung")
    p.add_argument("--view", action="store_true", help="Viewer zeigen (realtime)")
    args = p.parse_args()

    # HOLD_BASE-Konfiguration fuer diesen Test (ueberschreibt config.py-Modus).
    hb_cfg = types.SimpleNamespace(
        HOLD_BASE=(args.mode != "off"),
        HOLD_BASE_MODE=args.mode,
        HOLD_BASE_STIFFNESS=getattr(sim_config, "HOLD_BASE_STIFFNESS", 2000.0),
        HOLD_BASE_DAMPING=getattr(sim_config, "HOLD_BASE_DAMPING", 80.0),
    )

    model = mujoco.MjModel.from_xml_path(sim_config.ROBOT_SCENE)
    data = mujoco.MjData(model)
    model.opt.timestep = sim_config.SIMULATE_DT
    nu = model.nu

    hold = HoldBase(model, hb_cfg)

    target = np.zeros(nu)
    for k, idx in enumerate(LEFT_ARM):
        target[idx] = TARGET_LEFT[k]
    for k, idx in enumerate(RIGHT_ARM):
        target[idx] = TARGET_RIGHT[k]

    print(f"[hold_test] mode={args.mode} kp={args.kp:.0f} kd={args.kd:.0f} "
          f"dt={model.opt.timestep*1000:.1f}ms seconds={args.seconds}", flush=True)

    def apply_pd():
        # Nur Armmotoren stellen (wie arm_sdk); Beine/Taille macht HOLD_BASE.
        for idx in ARM:
            q = data.sensordata[idx]
            dq = data.sensordata[idx + nu]
            data.ctrl[idx] = args.kp * (target[idx] - q) + args.kd * (0.0 - dq)

    n_steps = int(args.seconds / model.opt.timestep)
    settle_steps = int(args.settle / model.opt.timestep)
    log_every = int(0.5 / model.opt.timestep)

    # Tracker fuer Schwingungsweite (min/max) je Armgelenk im Logfenster.
    win_min = {i: 1e9 for i in ARM}
    win_max = {i: -1e9 for i in ARM}
    worst_overall = 0.0

    viewer = None
    if args.view:
        viewer = mujoco.viewer.launch_passive(model, data)

    t0 = time.perf_counter()
    for step in range(n_steps):
        apply_pd()
        mujoco.mj_step(model, data)
        hold.after_step(data)

        for i in ARM:
            q = data.sensordata[i]
            if q < win_min[i]:
                win_min[i] = q
            if q > win_max[i]:
                win_max[i] = q

        if args.view and viewer is not None:
            viewer.sync()
            time.sleep(model.opt.timestep)

        if (step + 1) % log_every == 0:
            spans = {i: win_max[i] - win_min[i] for i in ARM}
            worst_i = max(spans, key=spans.get)
            worst = spans[worst_i]
            measuring = step >= settle_steps
            if measuring:
                worst_overall = max(worst_overall, worst)
            # Fehler ggue. Ziel (Mittel ueber Arme)
            errs = [abs(target[i] - data.sensordata[i]) for i in ARM]
            tag = "MESS" if measuring else "settle"
            print(f"[hold_test] t={ (step+1)*model.opt.timestep:5.2f}s {tag} "
                  f"osc_span_max={worst:.4f} rad (joint {worst_i})  "
                  f"mean|err|={np.mean(errs):.4f}", flush=True)
            win_min = {i: 1e9 for i in ARM}
            win_max = {i: -1e9 for i in ARM}

    wall = time.perf_counter() - t0
    print("=" * 64, flush=True)
    print(f"[hold_test] ERGEBNIS mode={args.mode}: groesste Schwingungsweite nach "
          f"Einschwingen = {worst_overall:.4f} rad", flush=True)
    if worst_overall < 0.01:
        print("[hold_test] -> STABIL. Die Physik haelt die Arme ruhig. Zittern im "
              "echten Lauf kommt dann aus der DDS-/Controller-Schicht.", flush=True)
    else:
        print("[hold_test] -> ZITTERT in reiner Physik. Ursache liegt in MuJoCo: "
              "HOLD_BASE-Modus / Gains / Integration. Vergleiche --mode und --kd.", flush=True)
    print(f"[hold_test] (sim {args.seconds:.0f}s in {wall:.1f}s Wandzeit, "
          f"{n_steps} Steps)", flush=True)


if __name__ == "__main__":
    main()
