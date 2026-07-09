#!/usr/bin/env python3
"""Validiert die KERN-Wette: steht die RL-Policy (motion.pt) bei cmd=0 mit
DETERMINISTISCHER 50-Hz-Physik (decimation=20, dt=0.001) auf eurem scene.xml?

Bildet die loco_sim-Pipeline 1:1 nach (Obs-Aufbau, Action->Target->PD je Sim-
Schritt wie die Bridge, Aktuator-Limits via MuJoCo). Kein DDS, kein Wall-Clock:
reiner In-Process-Lockstep = exakt das, was die echte Lockstep-Sim deterministisch
erzeugt. Wenn er HIER faellt, liegt es an Obs/Policy (nicht am Timing)."""
import numpy as np, math, mujoco, torch, yaml, os

_HERE = os.path.dirname(os.path.abspath(__file__))            # .../g1pilot
_ROOT = os.path.dirname(_HERE)                                # Repo-Wurzel
SCENE = os.path.join(_ROOT, "unitree_mujoco/unitree_robots/g1/scene.xml")
PDIR = os.path.join(_HERE, "policies/g1")
cfg = yaml.safe_load(open(os.path.join(PDIR, "g1.yaml")))

control_dt = cfg["control_dt"]; sim_dt = 0.001
decimation = int(round(control_dt / sim_dt))
leg_idx = cfg["leg_joint2motor_idx"]
kps = np.array(cfg["kps"]); kds = np.array(cfg["kds"])
default_angles = np.array(cfg["default_angles"])
aw_idx = cfg["arm_waist_joint2motor_idx"]
aw_kps = np.array(cfg["arm_waist_kps"]); aw_kds = np.array(cfg["arm_waist_kds"])
aw_target = np.array(cfg["arm_waist_target"])
ang_vel_scale = cfg["ang_vel_scale"]; dof_pos_scale = cfg["dof_pos_scale"]
dof_vel_scale = cfg["dof_vel_scale"]; action_scale = cfg["action_scale"]
cmd_scale = np.array(cfg["cmd_scale"]); max_cmd = np.array(cfg["max_cmd"])
num_actions = cfg["num_actions"]; num_obs = cfg["num_obs"]; gait_period = cfg["gait_period"]

torch._C._jit_set_profiling_executor(False)
policy = torch.jit.load(os.path.join(PDIR, cfg["policy_file"]), map_location="cpu").eval()
recurrent = hasattr(policy, "hidden_state") and hasattr(policy, "cell_state")
with torch.no_grad():
    for _ in range(5): policy(torch.zeros(1, num_obs))
if recurrent:
    policy.hidden_state.zero_(); policy.cell_state.zero_()
print(f"Policy: recurrent={recurrent}, decimation={decimation}, num_obs={num_obs}")


def grav(quat):
    qw, qx, qy, qz = quat
    return np.array([2*(-qz*qx+qw*qy), -2*(qz*qy+qw*qx), 1-2*(qw*qw+qz*qz)])


def reset_stance(m, d, pelvis_z=0.78):
    mujoco.mj_resetData(m, d)
    wid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_base_weld")
    if wid >= 0:
        m.eq_active0[wid] = 0; d.eq_active[wid] = 0
    qp = d.qpos
    qp[0]=0; qp[1]=0; qp[2]=pelvis_z; qp[3]=1; qp[4]=qp[5]=qp[6]=0
    for i in range(m.nu):
        qp[7+i] = default_angles[i] if i < 12 else 0.0
    d.qvel[:] = 0; mujoco.mj_forward(m, d)


def run(cmd=np.zeros(3), T=12.0, label=""):
    m = mujoco.MjModel.from_xml_path(SCENE); m.opt.timestep = sim_dt
    d = mujoco.MjData(m); reset_stance(m, d)
    if recurrent:
        policy.hidden_state.zero_(); policy.cell_state.zero_()
    n = m.nu
    action = np.zeros(num_actions)
    obs = np.zeros(num_obs)
    cmd_q = np.zeros(n); cmd_kp = np.zeros(n); cmd_kd = np.zeros(n)
    counter = 0
    nticks = int(T / control_dt)
    gx_hist = []
    pelvis_xy0 = d.qpos[:2].copy()
    for tick in range(nticks):
        # --- Obs (exakt wie loco_sim._build_obs) ---
        qj = np.array([d.sensordata[i] for i in leg_idx])
        dqj = np.array([d.sensordata[i+n] for i in leg_idx])
        quat = d.sensordata[3*n:3*n+4]
        ang_vel = d.sensordata[3*n+4:3*n+7].copy()
        g = grav(quat)
        counter += 1
        phase = (counter*control_dt % gait_period)/gait_period
        na = num_actions
        obs[:3] = ang_vel*ang_vel_scale
        obs[3:6] = g
        obs[6:9] = cmd*cmd_scale*max_cmd
        obs[9:9+na] = (qj-default_angles)*dof_pos_scale
        obs[9+na:9+2*na] = dqj*dof_vel_scale
        obs[9+2*na:9+3*na] = action
        obs[9+3*na] = math.sin(2*math.pi*phase)
        obs[9+3*na+1] = math.cos(2*math.pi*phase)
        with torch.no_grad():
            action = policy(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0)).numpy().squeeze()
        target = default_angles + action*action_scale
        for k, idx in enumerate(leg_idx):
            cmd_q[idx] = target[k]; cmd_kp[idx] = kps[k]; cmd_kd[idx] = kds[k]
        for j, idx in enumerate(aw_idx):
            cmd_q[idx] = aw_target[j]; cmd_kp[idx] = aw_kps[j]; cmd_kd[idx] = aw_kds[j]
        # --- decimation Physikschritte (Bridge-PD je Schritt) ---
        for _ in range(decimation):
            sd = d.sensordata
            for i in range(n):
                d.ctrl[i] = cmd_kp[i]*(cmd_q[i]-sd[i]) + cmd_kd[i]*(0.0-sd[i+n])
            mujoco.mj_step(m, d)
        gx = grav(d.sensordata[3*n:3*n+4])[0]
        gx_hist.append((tick*control_dt, gx, d.qpos[2]))
        if abs(gx) > 0.9 or d.qpos[2] < 0.4:
            drift = np.linalg.norm(d.qpos[:2]-pelvis_xy0)
            print(f"=== {label}: UMGEFALLEN bei t={tick*control_dt:.2f}s (gx={gx:+.2f}, z={d.qpos[2]:.2f}, drift={drift:.2f}m)")
            return False
    drift = np.linalg.norm(d.qpos[:2]-pelvis_xy0)
    print(f"=== {label}: STEHT {T:.0f}s (gx_final={gx:+.3f}, z={d.qpos[2]:.2f}, drift={drift:.2f}m)")
    print("    gx:", " ".join(f"t{t:.0f}={x:+.2f}" for t,x,_ in gx_hist[::max(1,len(gx_hist)//10)]))
    return True


if __name__ == "__main__":
    run(cmd=np.zeros(3), label="cmd=0 (station-keeping / BALANCE)")
    run(cmd=np.array([0.5, 0, 0]), label="cmd=[0.5,0,0] (vorwaerts laufen, Gegenprobe)")
