#!/usr/bin/env python3
"""Getreuer Headless-Test des aktuellen modellbasierten Balance-Reglers.

Bildet die echte Sim nach: scene.xml, dt=0.001, Reset wie die Bridge (Stand-Pose,
Pelvis z=0.78, Weld aus), Regelgesetz = loco_sim._send_balance EXAKT, PD pro
Sim-Schritt wie die Bridge (mit Aktuator-ctrlrange-Clamping durch MuJoCo).
Ziel: das Vorwaerts-Kippen aus dem echten Log reproduzieren und die Ursache messen.
"""
import os
import numpy as np
import mujoco

_HERE = os.path.dirname(os.path.abspath(__file__))            # .../g1pilot
_ROOT = os.path.dirname(_HERE)                                # Repo-Wurzel
SCENE = os.path.join(_ROOT, "unitree_mujoco/unitree_robots/g1/scene.xml")

# --- Config 1:1 ---
default_angles = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                           -0.1, 0.0, 0.0, 0.3, -0.2, 0.0], dtype=np.float64)
kps = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float64)
kds = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float64)
aw_idx = [12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28]
aw_kps = np.array([300, 300, 300, 100, 100, 50, 50, 20, 20, 20, 100, 100, 50, 50, 20, 20, 20], dtype=np.float64)
aw_kds = np.array([3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 2, 2, 2, 2, 1, 1, 1], dtype=np.float64)
aw_target = np.zeros(17)

L_HIP_PITCH, L_HIP_ROLL, L_ANKLE_PITCH, L_ANKLE_ROLL = 0, 1, 4, 5
R_HIP_PITCH, R_HIP_ROLL, R_ANKLE_PITCH, R_ANKLE_ROLL = 6, 7, 10, 11


def get_gravity_orientation(quat):
    qw, qx, qy, qz = quat
    g = np.zeros(3)
    g[0] = 2.0 * (-qz * qx + qw * qy)
    g[1] = -2.0 * (qz * qy + qw * qx)
    g[2] = 1.0 - 2.0 * (qw * qw + qz * qz)
    return g


def reset_stance(m, d, pelvis_z=0.78):
    mujoco.mj_resetData(m, d)
    # Weld deaktivieren (Basis frei)
    wid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_base_weld")
    if wid >= 0:
        m.eq_active0[wid] = 0
        d.eq_active[wid] = 0
    qp = d.qpos
    qp[0] = 0.0; qp[1] = 0.0; qp[2] = pelvis_z
    qp[3] = 1.0; qp[4] = 0.0; qp[5] = 0.0; qp[6] = 0.0
    for i in range(m.nu):
        qp[7 + i] = default_angles[i] if i < 12 else 0.0
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)


def measure_com_vs_feet(m, d):
    """CoM des ganzen Roboters vs. Fuss-/Knoechel-Position (x = vorn/hinten)."""
    com = d.subtree_com[0].copy()   # body 0 = world subtree = ganzer Roboter
    # Knoechel-/Fuss-Bodies
    lf = m.body("left_ankle_roll_link").id
    rf = m.body("right_ankle_roll_link").id
    foot_x = 0.5 * (d.xpos[lf][0] + d.xpos[rf][0])
    foot_z = 0.5 * (d.xpos[lf][2] + d.xpos[rf][2])
    return com, foot_x, foot_z


def run(bal_kp_scale=10.0, akp_p=150, akd_p=40, akp_r=150, akd_r=40,
        hkp_p=200, hkd_p=40, hkp_r=200, hkd_r=40, a_lim=50, h_lim=80,
        ctrl_hz=50.0, T=8.0, hold_arms=True, label=""):
    m = mujoco.MjModel.from_xml_path(SCENE)
    m.opt.timestep = 0.001
    d = mujoco.MjData(m)
    reset_stance(m, d)

    com0, fx0, fz0 = measure_com_vs_feet(m, d)
    print(f"\n=== {label} ===")
    print(f"  Start: CoM=({com0[0]:+.3f},{com0[1]:+.3f},{com0[2]:.3f})  "
          f"Fuss_x={fx0:+.3f} Fuss_z={fz0:.3f}  -> CoM_vor_Fuss={com0[0]-fx0:+.3f} m")

    n = m.nu
    steps_per_ctrl = int(round((1.0 / ctrl_hz) / m.opt.timestep))
    nsteps = int(T / m.opt.timestep)
    kd_scale = np.sqrt(max(bal_kp_scale, 1e-6))

    # aktuelle motor_cmd-Puffer (werden bei jedem Control-Tick neu gesetzt)
    cmd_q = np.zeros(n); cmd_kp = np.zeros(n); cmd_kd = np.zeros(n); cmd_tau = np.zeros(n)

    def control_tick():
        quat = d.sensordata[3*n:3*n+4]          # imu_quat
        gyro = d.sensordata[3*n+4:3*n+7].copy()  # imu_gyro
        g = get_gravity_orientation(quat)
        pitch_err, roll_err = g[0], g[1]
        pitch_rate, roll_rate = gyro[1], gyro[0]

        def clamp(x, lim):
            return max(-lim, min(lim, x))
        t_ap = clamp(akp_p*pitch_err + akd_p*pitch_rate, a_lim)
        t_ar = clamp(-(akp_r*roll_err + akd_r*roll_rate), a_lim)
        t_hp = clamp(hkp_p*pitch_err + hkd_p*pitch_rate, h_lim)
        t_hr = clamp(-(hkp_r*roll_err + hkd_r*roll_rate), h_lim)
        tau = np.zeros(12)
        tau[L_ANKLE_PITCH] = tau[R_ANKLE_PITCH] = t_ap
        tau[L_ANKLE_ROLL] = tau[R_ANKLE_ROLL] = t_ar
        tau[L_HIP_PITCH] = tau[R_HIP_PITCH] = t_hp
        tau[L_HIP_ROLL] = tau[R_HIP_ROLL] = t_hr
        # Beine
        for k in range(12):
            cmd_q[k] = default_angles[k]; cmd_tau[k] = tau[k]
            cmd_kp[k] = kps[k]*bal_kp_scale; cmd_kd[k] = kds[k]*kd_scale
        # Taille+Arme
        for j, idx in enumerate(aw_idx):
            sc = bal_kp_scale if idx in (12, 13, 14) else 1.0
            cmd_q[idx] = aw_target[j]; cmd_tau[idx] = 0.0
            cmd_kp[idx] = aw_kps[j]*sc; cmd_kd[idx] = aw_kds[j]*np.sqrt(sc)
        return g, tau

    g_log = []
    last_g = np.array([0, 0, -1.0])
    last_tau = np.zeros(12)
    for s in range(nsteps):
        if s % steps_per_ctrl == 0:
            last_g, last_tau = control_tick()
            g_log.append((s*m.opt.timestep, last_g.copy()))
        # Bridge-PD pro Sim-Schritt (mit aktuellen Sensoren)
        sd = d.sensordata
        for i in range(n):
            q = sd[i]; dq = sd[i+n]
            d.ctrl[i] = cmd_tau[i] + cmd_kp[i]*(cmd_q[i]-q) + cmd_kd[i]*(0.0-dq)
        mujoco.mj_step(m, d)
        # umgefallen?
        gx = get_gravity_orientation(d.sensordata[3*n:3*n+4])[0]
        if abs(gx) > 0.95:
            print(f"  UMGEFALLEN bei t={s*m.opt.timestep:.2f}s (gx={gx:+.2f})")
            break
    else:
        gx = get_gravity_orientation(d.sensordata[3*n:3*n+4])[0]
        print(f"  STEHT nach {T:.1f}s (gx={gx:+.3f})")

    # gx-Verlauf grob
    print("  gx-Verlauf:", " ".join(
        f"t{t:.1f}={gg[0]:+.2f}" for t, gg in g_log[::max(1, len(g_log)//10)]))
    return d


if __name__ == "__main__":
    m = mujoco.MjModel.from_xml_path(SCENE)
    m.opt.timestep = 0.001
    d = mujoco.MjData(m)
    reset_stance(m, d)
    com, fx, fz = measure_com_vs_feet(m, d)
    print(f"GEOMETRIE bei Stand-Pose (default_angles):")
    print(f"  CoM x={com[0]:+.4f}  Fuss-Mitte x={fx:+.4f}  -> CoM liegt {1000*(com[0]-fx):+.1f} mm vor der Fussmitte")
    print(f"  (Fuss reicht ~ -0.05..+0.13 m um die Knoechel-x; >+130mm = vor den Zehen = kippt)")

    # Test 1: aktueller Regler, Arme gehalten (=mein frueherer Test)
    run(label="AKTUELL kp_scale=10, Arme@0 (frueherer 'bestandener' Test)")
    # Test 2: ohne aktives tau (nur steifer Stand) — kippt er allein durch Geometrie?
    run(akp_p=0, akd_p=0, akp_r=0, akd_r=0, hkp_p=0, hkd_p=0, hkp_r=0, hkd_r=0,
        label="NUR steifer Stand, KEIN aktives tau (Geometrie-Check)")
    # Test 3: Vorzeichen umgedreht (Gegenprobe)
    run(akp_p=-150, hkp_p=-200, label="Vorzeichen PITCH UMGEDREHT (Gegenprobe)")
