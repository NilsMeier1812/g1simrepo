#!/usr/bin/env python3
"""PIPELINE-AUDIT: 1:1-Replik von unitree_rl_gym deploy_mujoco.py fuer motion.pt.

Baut die Observation GENAU wie der Referenz-Deploy (direkt aus qpos/qvel, NICHT
ueber die Bridge-Sensoren), mit den Referenz-Defaults simulation_dt=0.002 und
control_decimation=10. Wenn die Policy SO sauber steht/laeuft, liegt unser
Problem in der Sim-Umgebung (dt, Bridge, Timing) — nicht an Policy/Config.

Vergleicht zusaetzlich dt=0.002 vs unser dt=0.001.
"""
import numpy as np, math, mujoco, torch, yaml, os, sys

_HERE=os.path.dirname(os.path.abspath(__file__)); _ROOT=os.path.dirname(_HERE)
SCENE=os.path.join(_ROOT,"unitree_mujoco/unitree_robots/g1/scene.xml")
PDIR=os.path.join(_HERE,"policies/g1")
cfg=yaml.safe_load(open(os.path.join(PDIR,"g1.yaml")))

kps=np.array(cfg["kps"],dtype=float); kds=np.array(cfg["kds"],dtype=float)
default_angles=np.array(cfg["default_angles"],dtype=float)
aw_idx=cfg["arm_waist_joint2motor_idx"]
aw_kps=np.array(cfg["arm_waist_kps"],dtype=float); aw_kds=np.array(cfg["arm_waist_kds"],dtype=float)
aw_target=np.array(cfg["arm_waist_target"],dtype=float)
ang_vel_scale=cfg["ang_vel_scale"]; dof_pos_scale=cfg["dof_pos_scale"]
dof_vel_scale=cfg["dof_vel_scale"]; action_scale=cfg["action_scale"]
cmd_scale=np.array(cfg["cmd_scale"]); num_actions=cfg["num_actions"]; num_obs=cfg["num_obs"]
control_dt=cfg["control_dt"]; gait_period=cfg["gait_period"]; max_cmd=np.array(cfg["max_cmd"])

torch._C._jit_set_profiling_executor(False)
policy=torch.jit.load(os.path.join(PDIR,cfg["policy_file"]),map_location="cpu").eval()
recurrent=hasattr(policy,"hidden_state") and hasattr(policy,"cell_state")


def get_gravity_orientation(q):
    qw,qx,qy,qz=q
    return np.array([2*(-qz*qx+qw*qy), -2*(qz*qy+qw*qx), 1-2*(qw*qw+qz*qz)])


def pd_control(tq,q,kp,tdq,dq,kd): return (tq-q)*kp+(tdq-dq)*kd


def run(cmd_vec, sim_dt=0.002, T=10.0, label=""):
    m=mujoco.MjModel.from_xml_path(SCENE); m.opt.timestep=sim_dt
    d=mujoco.MjData(m)
    # Reset wie Bridge: aufrechte Stand-Pose, Weld aus
    mujoco.mj_resetData(m,d)
    wid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,"hold_base_weld")
    if wid>=0: m.eq_active0[wid]=0; d.eq_active[wid]=0
    d.qpos[2]=0.78; d.qpos[3]=1; d.qpos[4]=d.qpos[5]=d.qpos[6]=0
    for i in range(m.nu): d.qpos[7+i]=default_angles[i] if i<12 else 0.0
    d.qvel[:]=0; mujoco.mj_forward(m,d)
    if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()

    decim=int(round(control_dt/sim_dt))
    nj=m.nu  # 29
    action=np.zeros(num_actions); obs=np.zeros(num_obs,dtype=np.float32)
    target_dof=np.zeros(nj)
    # Soll fuer Arme/Taille fix
    for j,idx in enumerate(aw_idx): target_dof[idx]=aw_target[j]
    full_kp=np.zeros(nj); full_kd=np.zeros(nj)
    for k in range(12): full_kp[k]=kps[k]; full_kd[k]=kds[k]
    for j,idx in enumerate(aw_idx): full_kp[idx]=aw_kps[j]; full_kd[idx]=aw_kds[j]

    counter=0; start_xy=d.qpos[:2].copy(); cmd=np.array(cmd_vec,dtype=float)
    nsteps=int(T/sim_dt); gx_hist=[]
    for s in range(nsteps):
        # PD jeden Sim-Schritt (wie Referenz)
        q=d.qpos[7:7+nj]; dq=d.qvel[6:6+nj]
        tau=pd_control(target_dof,q,full_kp,np.zeros(nj),dq,full_kd)
        d.ctrl[:]=tau
        mujoco.mj_step(m,d)
        counter+=1
        if counter % decim==0:
            qj=(d.qpos[7:7+12]-default_angles)*dof_pos_scale
            dqj=d.qvel[6:6+12]*dof_vel_scale
            quat=d.qpos[3:7]; omega=d.qvel[3:6]*ang_vel_scale
            grav=get_gravity_orientation(quat)
            count=counter*sim_dt; phase=(count % gait_period)/gait_period
            obs[:3]=omega; obs[3:6]=grav; obs[6:9]=cmd*cmd_scale
            obs[9:9+12]=qj; obs[9+12:9+24]=dqj; obs[9+24:9+36]=action
            obs[9+36]=math.sin(2*math.pi*phase); obs[9+36+1]=math.cos(2*math.pi*phase)
            with torch.no_grad():
                action=policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
            target_dof[:12]=default_angles+action*action_scale
            gx=grav[0]
            if abs(gx)>0.9 or d.qpos[2]<0.5:
                drift=np.linalg.norm(d.qpos[:2]-start_xy)
                print(f"{label:<34} FIEL t={s*sim_dt:5.2f}s  drift={drift:.2f}m")
                return False
            gx_hist.append((s*sim_dt, gx, np.linalg.norm((d.qpos[:2]-start_xy))))
    drift=np.linalg.norm(d.qpos[:2]-start_xy)
    spd=np.linalg.norm(d.qvel[:2])
    print(f"{label:<34} steht {T:.0f}s  drift={drift:.2f}m  v_end={spd:.2f}m/s")
    return True


if __name__=="__main__":
    print("=== Referenz-Replik (unitree_rl_gym deploy_mujoco), Obs aus qpos/qvel ===\n")
    print("--- STEHEN (cmd=0) ---")
    run([0,0,0], sim_dt=0.002, label="cmd=0  dt=0.002 (Referenz)")
    run([0,0,0], sim_dt=0.001, label="cmd=0  dt=0.001 (unsere Sim)")
    run([0,0,0], sim_dt=0.005, label="cmd=0  dt=0.005")
    print("\n--- GEHEN (cmd=[0.5,0,0] vorwaerts) ---")
    run([0.5,0,0], sim_dt=0.002, label="cmd=0.5  dt=0.002 (Referenz)")
    run([0.5,0,0], sim_dt=0.001, label="cmd=0.5  dt=0.001 (unsere Sim)")
