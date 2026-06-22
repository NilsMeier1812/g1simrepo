#!/usr/bin/env python3
"""FROZEN-PHASE-TRICK: Kann die Geh-Policy bei cmd=0 STILL stehen, wenn man die
Gait-Phase (sin/cos) auf einen festen Wert einfriert (statt sie weiterlaufen zu
lassen)? Dann wuerde sie beide Fuesse planten statt auf der Stelle zu steppen
-> kein Drift, sauberes Stehen ohne Controller-Wechsel.

Walk 0.6 vorwaerts (3s, Phase laeuft) -> Release: cmd=0 UND Phase eingefroren auf
'pf'. Messe Drift & Reststand ueber 8s nach Release. Vergleich: pf=None (Phase
laeuft weiter, = heutiges Verhalten)."""
import numpy as np, math, mujoco, torch
import audit_reference_deploy as A
cfg=A.cfg; policy=A.policy; recurrent=A.recurrent
default_angles=A.default_angles; kps=A.kps; kds=A.kds
aw_idx=A.aw_idx; aw_kps=A.aw_kps; aw_kds=A.aw_kds; aw_target=A.aw_target
ang_vel_scale=A.ang_vel_scale; dof_pos_scale=A.dof_pos_scale; dof_vel_scale=A.dof_vel_scale
action_scale=A.action_scale; cmd_scale=A.cmd_scale; num_actions=A.num_actions; num_obs=A.num_obs
control_dt=A.control_dt; gait_period=A.gait_period; max_cmd=A.max_cmd
grav=A.get_gravity_orientation

def run(pf, walk_v=0.6, t_release=3.0, sim_dt=0.002, T=11.0):
    m=mujoco.MjModel.from_xml_path(A.SCENE); m.opt.timestep=sim_dt
    d=mujoco.MjData(m); mujoco.mj_resetData(m,d)
    wid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,"hold_base_weld")
    if wid>=0: m.eq_active0[wid]=0; d.eq_active[wid]=0
    d.qpos[2]=0.78; d.qpos[3]=1; d.qpos[4]=d.qpos[5]=d.qpos[6]=0
    for i in range(m.nu): d.qpos[7+i]=default_angles[i] if i<12 else 0.0
    d.qvel[:]=0; mujoco.mj_forward(m,d)
    if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
    nj=m.nu; decim=int(round(control_dt/sim_dt))
    action=np.zeros(num_actions); obs=np.zeros(num_obs,dtype=np.float32)
    target_dof=np.zeros(nj)
    for j,idx in enumerate(aw_idx): target_dof[idx]=aw_target[j]
    full_kp=np.zeros(nj); full_kd=np.zeros(nj)
    for j,idx in enumerate(aw_idx): full_kp[idx]=aw_kps[j]; full_kd[idx]=aw_kds[j]
    for k in range(12): full_kp[k]=kps[k]; full_kd[k]=kds[k]
    counter=0; xy_release=None
    nsteps=int(T/sim_dt)
    for s in range(nsteps):
        t=s*sim_dt
        if s % decim==0:
            quat=d.qpos[3:7]; omega=d.qvel[3:6]; g=grav(quat)
            walking = t<t_release
            cmd=np.array([walk_v,0,0]) if walking else np.zeros(3)
            if xy_release is None and not walking: xy_release=d.qpos[:2].copy()
            counter+=1; cmd_phys=cmd*max_cmd
            qj=(d.qpos[7:7+12]-default_angles)*dof_pos_scale; dqj=d.qvel[6:6+12]*dof_vel_scale
            if walking or pf is None:
                count=counter*control_dt; phase=(count%gait_period)/gait_period
                sp,cp=math.sin(2*math.pi*phase),math.cos(2*math.pi*phase)
            else:
                sp,cp=math.sin(2*math.pi*pf),math.cos(2*math.pi*pf)  # eingefroren
            obs[:3]=omega*ang_vel_scale; obs[3:6]=g; obs[6:9]=cmd_phys*cmd_scale
            obs[9:9+12]=qj; obs[9+12:9+24]=dqj; obs[9+24:9+36]=action
            obs[9+36]=sp; obs[9+36+1]=cp
            with torch.no_grad():
                action=policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
            target_dof[:12]=default_angles+action*action_scale
        q=d.qpos[7:7+nj]; dq=d.qvel[6:6+nj]
        d.ctrl[:]=(target_dof-q)*full_kp+(0-dq)*full_kd
        mujoco.mj_step(m,d)
        if abs(grav(d.qpos[3:7])[0])>0.9 or d.qpos[2]<0.5:
            return dict(fell=True,t=t,pf=pf)
    drift_rel=np.linalg.norm(d.qpos[:2]-xy_release) if xy_release is not None else 0.0
    return dict(fell=False,pf=pf,drift_rel=drift_rel,v_end=np.linalg.norm(d.qvel[:2]))

if __name__=="__main__":
    print("Walk 0.6 (3s) -> Release: cmd=0, Phase eingefroren auf pf. Drift ueber 8s n.Release.\n")
    print(f"{'phase_freeze':<16}{'Ergebnis':<18}{'Drift n.Release':<18}{'v_end'}")
    print("-"*64)
    print(f"{'(laeuft weiter)':<16}", end="")
    r=run(None); print(f"{'GEFALLEN t=%.1f'%r['t'] if r['fell'] else 'steht':<18}{'' if r['fell'] else '%.3f m'%r['drift_rel']:<18}{'' if r['fell'] else '%.3f'%r['v_end']}")
    for pf in (0.0,0.125,0.25,0.375,0.5,0.625,0.75,0.875):
        r=run(pf)
        if r["fell"]: print(f"{pf:<16}{'GEFALLEN t=%.1f'%r['t']:<18}")
        else: print(f"{pf:<16}{'steht':<18}{r['drift_rel']:<18.3f}{r['v_end']:.3f}")
