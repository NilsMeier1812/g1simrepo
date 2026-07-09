#!/usr/bin/env python3
"""Headless Push-Recovery-Vergleich: Wie viel Stoss steckt der PD-Balancer weg,
und wie viel die RL-Policy (motion.pt)? Findet per Bisektion die maximale
wegsteckbare Kraft (120 ms Impuls auf torso_link) — vorwaerts und seitlich.

Beide Regelgesetze sind 1:1 aus test_balance_faithful.py (PD) bzw.
test_policy_lockstep.py (Policy) uebernommen. Reiner In-Process-Lockstep,
deterministisch (dt=0.001, decimation=20). Kein DDS/Wall-Clock.
"""
import os, math, numpy as np, mujoco, torch, yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
SCENE = os.path.join(_ROOT, "unitree_mujoco/unitree_robots/g1/scene.xml")
PDIR = os.path.join(_HERE, "policies/g1")

PUSH_T0 = 2.0        # Stoss-Start [s] (erst stabil stehen lassen)
PUSH_DUR = 0.12      # Stoss-Dauer [s] (wie real)
SIM_T = 6.0          # Gesamtdauer [s]
sim_dt = 0.001

# --- PD-Config (Defaults wie loco_sim/test_balance_faithful) ---
default_angles = np.array([-0.1,0,0,0.3,-0.2,0, -0.1,0,0,0.3,-0.2,0], dtype=np.float64)
kps_pd = np.array([100,100,100,150,40,40,100,100,100,150,40,40], dtype=np.float64)
kds_pd = np.array([2,2,2,4,2,2,2,2,2,4,2,2], dtype=np.float64)
aw_idx = [12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28]
aw_kps = np.array([300,300,300,100,100,50,50,20,20,20,100,100,50,50,20,20,20], dtype=np.float64)
aw_kds = np.array([3,3,3,2,2,2,2,1,1,1,2,2,2,2,1,1,1], dtype=np.float64)
L_HIP_PITCH,L_HIP_ROLL,L_ANKLE_PITCH,L_ANKLE_ROLL = 0,1,4,5
R_HIP_PITCH,R_HIP_ROLL,R_ANKLE_PITCH,R_ANKLE_ROLL = 6,7,10,11
BAL_KP=10.0; AKP_P=150; AKD_P=40; AKP_R=150; AKD_R=40
HKP_P=200; HKD_P=40; HKP_R=200; HKD_R=40; A_LIM=50; H_LIM=80


def grav(quat):
    qw,qx,qy,qz = quat
    return np.array([2*(-qz*qx+qw*qy), -2*(qz*qy+qw*qx), 1-2*(qw*qw+qz*qz)])

def reset_stance(m, d, pelvis_z=0.78):
    mujoco.mj_resetData(m, d)
    wid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_base_weld")
    if wid >= 0: m.eq_active0[wid]=0; d.eq_active[wid]=0
    qp = d.qpos
    qp[0]=0; qp[1]=0; qp[2]=pelvis_z; qp[3]=1; qp[4]=qp[5]=qp[6]=0
    for i in range(m.nu): qp[7+i] = default_angles[i] if i < 12 else 0.0
    d.qvel[:]=0; mujoco.mj_forward(m, d)

def fallen(m, d, n):
    g = grav(d.sensordata[3*n:3*n+4])
    return abs(g[0])>0.95 or abs(g[1])>0.95 or d.qpos[2]<0.45


def push_vec(force, direction):
    return np.array([force,0,0,0,0,0]) if direction=="vorne" else np.array([0,force,0,0,0,0])


def run_pd(force, direction):
    m = mujoco.MjModel.from_xml_path(SCENE); m.opt.timestep=sim_dt
    d = mujoco.MjData(m); reset_stance(m, d)
    n = m.nu; tb = m.body("torso_link").id
    kd_scale = math.sqrt(BAL_KP)
    cmd_q=np.zeros(n); cmd_kp=np.zeros(n); cmd_kd=np.zeros(n); cmd_tau=np.zeros(n)
    steps_per_ctrl = int(round(0.02/sim_dt)); nsteps=int(SIM_T/sim_dt)
    def clamp(x,l): return max(-l,min(l,x))
    def tick():
        quat=d.sensordata[3*n:3*n+4]; gyro=d.sensordata[3*n+4:3*n+7]
        g=grav(quat); pe,re=g[0],g[1]; pr,rr=gyro[1],gyro[0]
        t_ap=clamp(AKP_P*pe+AKD_P*pr,A_LIM); t_ar=clamp(-(AKP_R*re+AKD_R*rr),A_LIM)
        t_hp=clamp(HKP_P*pe+HKD_P*pr,H_LIM); t_hr=clamp(-(HKP_R*re+HKD_R*rr),H_LIM)
        tau=np.zeros(12)
        tau[L_ANKLE_PITCH]=tau[R_ANKLE_PITCH]=t_ap; tau[L_ANKLE_ROLL]=tau[R_ANKLE_ROLL]=t_ar
        tau[L_HIP_PITCH]=tau[R_HIP_PITCH]=t_hp; tau[L_HIP_ROLL]=tau[R_HIP_ROLL]=t_hr
        for k in range(12):
            cmd_q[k]=default_angles[k]; cmd_tau[k]=tau[k]
            cmd_kp[k]=kps_pd[k]*BAL_KP; cmd_kd[k]=kds_pd[k]*kd_scale
        for j,idx in enumerate(aw_idx):
            sc=BAL_KP if idx in (12,13,14) else 1.0
            cmd_q[idx]=0.0; cmd_tau[idx]=0.0
            cmd_kp[idx]=aw_kps[j]*sc; cmd_kd[idx]=aw_kds[j]*math.sqrt(sc)
    pv = push_vec(force, direction)
    for s in range(nsteps):
        if s % steps_per_ctrl == 0: tick()
        t = s*sim_dt
        d.xfrc_applied[tb] = pv if (PUSH_T0 <= t < PUSH_T0+PUSH_DUR) else 0.0
        sd=d.sensordata
        for i in range(n):
            d.ctrl[i]=cmd_tau[i]+cmd_kp[i]*(cmd_q[i]-sd[i])+cmd_kd[i]*(0.0-sd[i+n])
        mujoco.mj_step(m, d)
        if fallen(m, d, n): return False
    return True


# --- Policy laden ---
cfg = yaml.safe_load(open(os.path.join(PDIR,"g1.yaml")))
control_dt=cfg["control_dt"]; decimation=int(round(control_dt/sim_dt))
leg_idx=cfg["leg_joint2motor_idx"]; kps_p=np.array(cfg["kps"]); kds_p=np.array(cfg["kds"])
da=np.array(cfg["default_angles"]); awi=cfg["arm_waist_joint2motor_idx"]
awk=np.array(cfg["arm_waist_kps"]); awd=np.array(cfg["arm_waist_kds"]); awt=np.array(cfg["arm_waist_target"])
avs=cfg["ang_vel_scale"]; dps=cfg["dof_pos_scale"]; dvs=cfg["dof_vel_scale"]; ascl=cfg["action_scale"]
cmd_scale=np.array(cfg["cmd_scale"]); max_cmd=np.array(cfg["max_cmd"])
na=cfg["num_actions"]; no=cfg["num_obs"]; gait=cfg["gait_period"]
torch._C._jit_set_profiling_executor(False)
policy=torch.jit.load(os.path.join(PDIR,cfg["policy_file"]),map_location="cpu").eval()
recurrent=hasattr(policy,"hidden_state") and hasattr(policy,"cell_state")
with torch.no_grad():
    for _ in range(5): policy(torch.zeros(1,no))


def run_policy(force, direction, cmd=np.zeros(3)):
    m = mujoco.MjModel.from_xml_path(SCENE); m.opt.timestep=sim_dt
    d = mujoco.MjData(m); reset_stance(m, d)
    if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
    n=m.nu; tb=m.body("torso_link").id
    action=np.zeros(na); obs=np.zeros(no)
    cmd_q=np.zeros(n); cmd_kp=np.zeros(n); cmd_kd=np.zeros(n)
    counter=0; nticks=int(SIM_T/control_dt); pv=push_vec(force,direction)
    for tick in range(nticks):
        qj=np.array([d.sensordata[i] for i in leg_idx])
        dqj=np.array([d.sensordata[i+n] for i in leg_idx])
        quat=d.sensordata[3*n:3*n+4]; av=d.sensordata[3*n+4:3*n+7].copy(); g=grav(quat)
        counter+=1; phase=(counter*control_dt % gait)/gait
        obs[:3]=av*avs; obs[3:6]=g; obs[6:9]=cmd*cmd_scale*max_cmd
        obs[9:9+na]=(qj-da)*dps; obs[9+na:9+2*na]=dqj*dvs; obs[9+2*na:9+3*na]=action
        obs[9+3*na]=math.sin(2*math.pi*phase); obs[9+3*na+1]=math.cos(2*math.pi*phase)
        with torch.no_grad():
            action=policy(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0)).numpy().squeeze()
        target=da+action*ascl
        for k,idx in enumerate(leg_idx): cmd_q[idx]=target[k]; cmd_kp[idx]=kps_p[k]; cmd_kd[idx]=kds_p[k]
        for j,idx in enumerate(awi): cmd_q[idx]=awt[j]; cmd_kp[idx]=awk[j]; cmd_kd[idx]=awd[j]
        t0=tick*control_dt
        for step in range(decimation):
            t=t0+step*sim_dt
            d.xfrc_applied[tb] = pv if (PUSH_T0 <= t < PUSH_T0+PUSH_DUR) else 0.0
            sd=d.sensordata
            for i in range(n): d.ctrl[i]=cmd_kp[i]*(cmd_q[i]-sd[i])+cmd_kd[i]*(0.0-sd[i+n])
            mujoco.mj_step(m, d)
        if fallen(m, d, n): return False
    return True


def run_hybrid(force, direction, switch_tilt=0.25, warm=True):
    """Dein Vorschlag: PD-Balancer steht; sobald die Neigung switch_tilt
    ueberschreitet (= 'droht zu fallen'), wird auf die RL-Policy umgeschaltet
    (Stepping). warm=True: die Policy-LSTM laeuft im Hintergrund mit (Obs jeden
    Tick), wird aber erst nach dem Umschalten angewandt -> kein Kaltstart."""
    m = mujoco.MjModel.from_xml_path(SCENE); m.opt.timestep=sim_dt
    d = mujoco.MjData(m); reset_stance(m, d)
    if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
    n=m.nu; tb=m.body("torso_link").id
    kd_scale=math.sqrt(BAL_KP)
    cmd_q=np.zeros(n); cmd_kp=np.zeros(n); cmd_kd=np.zeros(n); cmd_tau=np.zeros(n)
    action=np.zeros(na); obs=np.zeros(no)
    steps_per_ctrl=int(round(0.02/sim_dt)); nticks=int(SIM_T/control_dt)
    counter=0; switched=False; pv=push_vec(force,direction)
    def clamp(x,l): return max(-l,min(l,x))
    for tick in range(nticks):
        quat=d.sensordata[3*n:3*n+4]; gyro=d.sensordata[3*n+4:3*n+7].copy(); g=grav(quat)
        counter+=1
        if not switched and (abs(g[0])>switch_tilt or abs(g[1])>switch_tilt):
            switched=True
        qj=np.array([d.sensordata[i] for i in leg_idx]); dqj=np.array([d.sensordata[i+n] for i in leg_idx])
        phase=(counter*control_dt % gait)/gait
        obs[:3]=gyro*avs; obs[3:6]=g; obs[6:9]=0.0
        obs[9:9+na]=(qj-da)*dps; obs[9+na:9+2*na]=dqj*dvs; obs[9+2*na:9+3*na]=action
        obs[9+3*na]=math.sin(2*math.pi*phase); obs[9+3*na+1]=math.cos(2*math.pi*phase)
        if warm or switched:
            with torch.no_grad():
                action=policy(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0)).numpy().squeeze()
        if not switched:
            pe,re=g[0],g[1]; pr,rr=gyro[1],gyro[0]
            t_ap=clamp(AKP_P*pe+AKD_P*pr,A_LIM); t_ar=clamp(-(AKP_R*re+AKD_R*rr),A_LIM)
            t_hp=clamp(HKP_P*pe+HKD_P*pr,H_LIM); t_hr=clamp(-(HKP_R*re+HKD_R*rr),H_LIM)
            tau=np.zeros(12)
            tau[L_ANKLE_PITCH]=tau[R_ANKLE_PITCH]=t_ap; tau[L_ANKLE_ROLL]=tau[R_ANKLE_ROLL]=t_ar
            tau[L_HIP_PITCH]=tau[R_HIP_PITCH]=t_hp; tau[L_HIP_ROLL]=tau[R_HIP_ROLL]=t_hr
            for k in range(12):
                cmd_q[k]=default_angles[k]; cmd_tau[k]=tau[k]
                cmd_kp[k]=kps_pd[k]*BAL_KP; cmd_kd[k]=kds_pd[k]*kd_scale
            for j,idx in enumerate(aw_idx):
                sc=BAL_KP if idx in (12,13,14) else 1.0
                cmd_q[idx]=0.0; cmd_tau[idx]=0.0
                cmd_kp[idx]=aw_kps[j]*sc; cmd_kd[idx]=aw_kds[j]*math.sqrt(sc)
        else:
            target=da+action*ascl
            for k,idx in enumerate(leg_idx): cmd_q[idx]=target[k]; cmd_kp[idx]=kps_p[k]; cmd_kd[idx]=kds_p[k]; cmd_tau[idx]=0.0
            for j,idx in enumerate(awi): cmd_q[idx]=awt[j]; cmd_kp[idx]=awk[j]; cmd_kd[idx]=awd[j]; cmd_tau[idx]=0.0
        t0=tick*control_dt
        for step in range(steps_per_ctrl):
            t=t0+step*sim_dt
            d.xfrc_applied[tb] = pv if (PUSH_T0 <= t < PUSH_T0+PUSH_DUR) else 0.0
            sd=d.sensordata
            for i in range(n): d.ctrl[i]=cmd_tau[i]+cmd_kp[i]*(cmd_q[i]-sd[i])+cmd_kd[i]*(0.0-sd[i+n])
            mujoco.mj_step(m, d)
        if fallen(m, d, n): return False
    return True


def max_recoverable(simfn, direction, lo=0, hi=400, tol=15, **kw):
    # erst pruefen, ob hi schon ueberlebt wird
    if simfn(hi, direction, **kw): return hi, True
    if not simfn(lo, direction, **kw): return lo, False
    while hi-lo > tol:
        mid=(lo+hi)/2
        if simfn(mid, direction, **kw): lo=mid
        else: hi=mid
    return lo, False


if __name__ == "__main__":
    mass = mujoco.MjModel.from_xml_path(SCENE)
    md = mujoco.MjData(mass); reset_stance(mass, md)
    total_mass = mass.body_subtreemass[0]
    print(f"Roboter-Gesamtmasse: {total_mass:.1f} kg | Impuls = F*{PUSH_DUR}s")
    print(f"{'Regler':<10}{'Richtung':<10}{'max F [N]':>10}{'Impuls [Ns]':>13}{'~dv [m/s]':>11}")
    print("-"*54)
    for name, fn in (("PD", run_pd), ("Policy", run_policy), ("Hybrid", run_hybrid)):
        for direction in ("vorne", "seite"):
            F, _ = max_recoverable(fn, direction)
            imp = F*PUSH_DUR; dv = imp/total_mass
            print(f"{name:<10}{direction:<10}{F:>10.0f}{imp:>13.1f}{dv:>11.2f}")
