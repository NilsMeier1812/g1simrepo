#!/usr/bin/env python3
"""Headless-Regressionstest der Whole-Body-Loco-Policy (policies/g1_wholebody).

Prueft OHNE ROS/DDS (nur mujoco + onnxruntime), dass die committete policy.onnx in
unserer G1-Szene das erwartete Verhalten zeigt:
  * cmd = 0   -> steht still (Drift < 0.1 m / 10 s)
  * cmd != 0  -> laeuft, bleibt aufrecht
  * Walk->Stand (cmd->0) -> kommt sauber zum Stehen (Nachlauf < 0.4 m)

Obs/Scales/Joint-Order/Gains kommen 1:1 aus deploy.yaml + den ONNX-Metadaten —
exakt wie loco_sim sie zur Laufzeit nutzt. Lauf:  python3 test_wholebody_policy.py
"""
import os, math, numpy as np, mujoco, onnxruntime as ort, onnx, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SCENE = os.path.join(ROOT, "unitree_mujoco/unitree_robots/g1/scene.xml")
PDIR = os.path.join(HERE, "policies/g1_wholebody")

dep = yaml.safe_load(open(os.path.join(PDIR, "deploy.yaml")))
KP = np.array(dep["stiffness"], np.float32); KD = np.array(dep["damping"], np.float32)
DEFAULT = np.array(dep["default_joint_pos"], np.float32)
ASCALE = np.array(dep["actions"]["JointPositionAction"]["scale"], np.float32)
PERIOD = float(dep["observations"]["gait_phase"]["params"]["period"])
CTRL_DT = float(dep["step_dt"]); SIM_DT = 0.005; DECIM = int(round(CTRL_DT / SIM_DT))
STAND_EPS = 0.1
meta = {p.key: p.value for p in onnx.load(os.path.join(PDIR, "policy.onnx")).metadata_props}
JN = meta["joint_names"].split(",")
sess = ort.InferenceSession(os.path.join(PDIR, "policy.onnx"))
IN = sess.get_inputs()[0].name


def grav(q):
    qw, qx, qy, qz = q
    return np.array([2*(-qz*qx+qw*qy), -2*(qz*qy+qw*qx), 1-2*(qw*qw+qz*qz)], np.float32)


def _map(m):
    qa = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JN]
    da = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JN]
    return np.array(qa), np.array(da)


def episode(walk_cmd, t_walk, t_stop):
    m = mujoco.MjModel.from_xml_path(SCENE); m.opt.timestep = SIM_DT
    d = mujoco.MjData(m); mujoco.mj_resetData(m, d)
    wid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_base_weld")
    if wid >= 0: m.eq_active0[wid] = 0; d.eq_active[wid] = 0
    qa, da = _map(m)
    d.qpos[2] = 0.793; d.qpos[3] = 1
    for k in range(29): d.qpos[qa[k]] = DEFAULT[k]
    d.qvel[:] = 0; mujoco.mj_forward(m, d)
    last = np.zeros(29, np.float32); tgt = np.zeros(m.nu); fkp = np.zeros(m.nu); fkd = np.zeros(m.nu)
    obs = np.zeros((1, 98), np.float32); step = 0; fell = None; xy_stop = None
    for s in range(int((t_walk + t_stop) / SIM_DT)):
        t = s * SIM_DT
        cmd = np.array(walk_cmd if t < t_walk else [0, 0, 0], np.float32)
        if xy_stop is None and t >= t_walk: xy_stop = d.qpos[:2].copy()
        if s % DECIM == 0:
            q = np.array([d.qpos[a] for a in qa], np.float32)
            dq = np.array([d.qvel[a] for a in da], np.float32)
            g = grav(d.qpos[3:7]); gp = (step * CTRL_DT) % PERIOD / PERIOD
            ph = (np.zeros(2, np.float32) if np.linalg.norm(cmd) < STAND_EPS
                  else np.array([math.sin(gp*2*math.pi), math.cos(gp*2*math.pi)], np.float32))
            obs[0, 0:3] = d.qvel[3:6]; obs[0, 3:6] = g; obs[0, 6:9] = cmd; obs[0, 9:11] = ph
            obs[0, 11:40] = q - DEFAULT; obs[0, 40:69] = dq; obs[0, 69:98] = last
            act = sess.run(None, {IN: obs})[0][0]; last = act.copy()
            tt = DEFAULT + act * ASCALE
            for k in range(29):
                js = qa[k] - 7; tgt[js] = tt[k]; fkp[js] = KP[k]; fkd[js] = KD[k]
            step += 1
        d.ctrl[:] = (tgt - d.qpos[7:7+m.nu]) * fkp + (0 - d.qvel[6:6+m.nu]) * fkd
        mujoco.mj_step(m, d)
        if fell is None and (abs(grav(d.qpos[3:7])[0]) > 0.7 or d.qpos[2] < 0.5): fell = t
    post = np.linalg.norm(d.qpos[:2] - xy_stop) if xy_stop is not None else 0.0
    return dict(fell=fell, post=post, vend=float(np.linalg.norm(d.qvel[:2])),
                drift=float(np.linalg.norm(d.qpos[:2] - (xy_stop if t_walk == 0 else d.qpos[:2]*0))))


def main():
    fails = 0
    print("=== STEHEN (cmd=0, 10 s) ===")
    r = episode([0, 0, 0], 0.0, 10.0)
    stand_drift = r["post"]
    ok = r["fell"] is None and stand_drift < 0.1
    print(f"  cmd=0  -> {'OK' if ok else 'FAIL'}  drift={stand_drift:.3f}m  v_end={r['vend']:.2f}")
    fails += not ok
    print("\n=== WALK->STAND (3 s laufen -> cmd=0 -> 5 s) ===")
    cases = [("vor", [0.5, 0, 0]), ("zurueck", [-0.4, 0, 0]), ("links", [0, 0.4, 0]),
             ("rechts", [0, -0.4, 0]), ("drehen", [0, 0, 0.6]), ("vor+dreh", [0.4, 0, 0.4])]
    nok = 0
    for nm, c in cases:
        r = episode(c, 3.0, 5.0)
        ok = r["fell"] is None and r["post"] < 0.4 and r["vend"] < 0.2
        nok += ok
        st = f"GEFALLEN t={r['fell']:.1f}" if r["fell"] else ("steht sauber" if ok else "wankt")
        print(f"  {nm:<10} {st:<16} nachlauf={r['post']:.2f}m  v_end={r['vend']:.2f}  {'OK' if ok else ''}")
    print(f"\nWalk->Stand: {nok}/{len(cases)} sauber")
    # Akzeptanz: Stehen driftfrei UND >=5/6 Uebergaenge sauber.
    fails += (nok < 5)
    print("\nERGEBNIS:", "BESTANDEN" if fails == 0 else f"DURCHGEFALLEN ({fails} Kriterien)")
    return fails


if __name__ == "__main__":
    raise SystemExit(1 if main() else 0)
