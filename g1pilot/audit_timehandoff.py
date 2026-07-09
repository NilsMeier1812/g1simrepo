#!/usr/bin/env python3
"""Wie audit_walk_stop_clean, aber Handoff REIN ZEITBASIERT (cmd=0 fuer settle_s
nach Stop -> PD). Das ist exakt, was loco_sim kann (keine Basis-Velocity verfuegbar)."""
import numpy as np, math, mujoco, torch
import audit_reference_deploy as A
cfg=A.cfg; policy=A.policy; recurrent=A.recurrent
default_angles=A.default_angles; kps=A.kps; kds=A.kds
ang_vel_scale=A.ang_vel_scale; dof_pos_scale=A.dof_pos_scale; dof_vel_scale=A.dof_vel_scale
action_scale=A.action_scale; cmd_scale=A.cmd_scale; num_actions=A.num_actions; num_obs=A.num_obs
control_dt=A.control_dt; gait_period=A.gait_period; grav=A.get_gravity_orientation
aw_idx=A.aw_idx; aw_kps=A.aw_kps; aw_kds=A.aw_kds; aw_target=A.aw_target
BAL_KP=10.0; AKP_P=150; AKD_P=40; AKP_R=150; AKD_R=40; HKP_P=200; HKD_P=40; HKP_R=200; HKD_R=40
A_LIM=50; H_LIM=80
kps_pd=np.array([100,100,100,150,40,40,100,100,100,150,40,40],dtype=float)
kds_pd=np.array([2,2,2,4,2,2,2,2,2,4,2,2],dtype=float)
L_HIP_PITCH,L_HIP_ROLL,L_ANKLE_PITCH,L_ANKLE_ROLL=0,1,4,5
R_HIP_PITCH,R_HIP_ROLL,R_ANKLE_PITCH,R_ANKLE_ROLL=6,7,10,11

def run(sim_dt=0.002, walk_v=0.5, t_walk=(2.0,6.0), settle_s=1.3, T=14.0):
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
    kd_scale=math.sqrt(BAL_KP)
    mode="PD"; counter=0; start_xy=d.qpos[:2].copy(); stop_t=None
    def cl(x,l): return max(-l,min(l,x))
    nsteps=int(T/sim_dt); ctrl_tau=np.zeros(12)
    for s in range(nsteps):
        t=s*sim_dt
        if s%decim==0:
            quat=d.qpos[3:7]; omega=d.qvel[3:6]; g=grav(quat)
            if mode=="PD" and t_walk[0]<=t<t_walk[1] and stop_t is None:
                mode="POL"
                if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
                action[:]=0.0
            if mode=="POL" and stop_t is None and t>=t_walk[1]: stop_t=t
            # ZEITBASIERTER Handoff: settle_s nach Stop -> PD
            if mode=="POL" and stop_t is not None and (t-stop_t)>settle_s: mode="PD"
            if mode=="POL":
                counter+=1
                cmd=np.array([walk_v,0,0.0]) if stop_t is None else np.zeros(3)
                qj=(d.qpos[7:7+12]-default_angles)*dof_pos_scale; dqj=d.qvel[6:6+12]*dof_vel_scale
                count=counter*control_dt; phase=(count%gait_period)/gait_period
                obs[:3]=omega*ang_vel_scale; obs[3:6]=g; obs[6:9]=cmd*cmd_scale
                obs[9:9+12]=qj; obs[9+12:9+24]=dqj; obs[9+24:9+36]=action
                obs[9+36]=math.sin(2*math.pi*phase); obs[9+36+1]=math.cos(2*math.pi*phase)
                with torch.no_grad():
                    action=policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
                target_dof[:12]=default_angles+action*action_scale
                for k in range(12): full_kp[k]=kps[k]; full_kd[k]=kds[k]
            else:
                pe,re=g[0],g[1]; pr,rr=omega[1],omega[0]
                tap=cl(AKP_P*pe+AKD_P*pr,A_LIM); tar=cl(-(AKP_R*re+AKD_R*rr),A_LIM)
                thp=cl(HKP_P*pe+HKD_P*pr,H_LIM); thr=cl(-(HKP_R*re+HKD_R*rr),H_LIM)
                ctrl_tau[:]=0
                ctrl_tau[L_ANKLE_PITCH]=ctrl_tau[R_ANKLE_PITCH]=tap; ctrl_tau[L_ANKLE_ROLL]=ctrl_tau[R_ANKLE_ROLL]=tar
                ctrl_tau[L_HIP_PITCH]=ctrl_tau[R_HIP_PITCH]=thp; ctrl_tau[L_HIP_ROLL]=ctrl_tau[R_HIP_ROLL]=thr
                target_dof[:12]=default_angles
                for k in range(12): full_kp[k]=kps_pd[k]*BAL_KP; full_kd[k]=kds_pd[k]*kd_scale
        q=d.qpos[7:7+nj]; dq=d.qvel[6:6+nj]
        tau=(target_dof-q)*full_kp+(0-dq)*full_kd
        if mode=="PD": tau[:12]+=ctrl_tau
        d.ctrl[:]=tau; mujoco.mj_step(m,d)
        if abs(grav(d.qpos[3:7])[0])>0.9 or d.qpos[2]<0.5:
            return dict(fell=True,t=t,drift=np.linalg.norm(d.qpos[:2]-start_xy),mode=mode)
    return dict(fell=False,drift=np.linalg.norm(d.qpos[:2]-start_xy),
                v_end=np.linalg.norm(d.qvel[:2]),mode=mode,handed=(mode=="PD"))

if __name__=="__main__":
    print("ZEITBASIERTER Handoff (was loco_sim kann). Stehen->Gehen0.5->cmd=0->[settle]->PD\n")
    print(f"{'settle_s':<10}{'sim_dt':<10}{'Ergebnis':<26}{'Endstand'}")
    print("-"*70)
    for settle in (1.0,1.3,1.6,2.0):
        for sdt in (0.002,0.001):
            r=run(sim_dt=sdt,settle_s=settle)
            if r["fell"]: res=f"GEFALLEN t={r['t']:.1f}s({r['mode']})"; end=f"drift={r['drift']:.2f}m"
            else:
                res="steht am PD" if r["handed"] else f"laeuft({r['mode']})"
                end=f"drift={r['drift']:.2f}m v={r['v_end']:.2f}"
            print(f"{settle:<10}{sdt:<10}{res:<26}{end}")
