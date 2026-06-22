#!/usr/bin/env python3
"""Beweist: DOPPELSTUETZ-getriggerter Handoff (beide Fuesse belastet + ruhige
Drehrate) ist robust ueber BELIEBIGE Release-Zeitpunkte — anders als zeitbasiert.

Continuous walk(0.6) -> Release@t_rel -> Policy laeuft cmd=0 weiter, bis BEIDE
Fuesse Bodenkontakt (Normalkraft>FTH) haben UND |gyro|<GYR -> dann Handoff an PD
(mit Rampe). Fuss-Normalkraft via MuJoCo-Kontakte = exakt das, was ein Fuss-
kraftsensor am echten G1 liefert (unitree_hg LowState_.foot_force).

Sweep ueber t_rel (verschiedene Gangphasen beim Loslassen) -> faellt es je?"""
import numpy as np, math, mujoco, torch
import audit_reference_deploy as A
import diag_gating as G
cfg=A.cfg; policy=A.policy; recurrent=A.recurrent
default_angles=A.default_angles; kps=A.kps; kds=A.kds
aw_idx=A.aw_idx; aw_kps=A.aw_kps; aw_kds=A.aw_kds; aw_target=A.aw_target
ang_vel_scale=A.ang_vel_scale; dof_pos_scale=A.dof_pos_scale; dof_vel_scale=A.dof_vel_scale
action_scale=A.action_scale; cmd_scale=A.cmd_scale; num_actions=A.num_actions; num_obs=A.num_obs
control_dt=A.control_dt; gait_period=A.gait_period; max_cmd=A.max_cmd
grav=A.get_gravity_orientation
kps_pd=G.kps_pd; kds_pd=G.kds_pd; BAL_KP=G.BAL_KP
AKP_P=G.AKP_P;AKD_P=G.AKD_P;AKP_R=G.AKP_R;AKD_R=G.AKD_R
HKP_P=G.HKP_P;HKD_P=G.HKD_P;HKP_R=G.HKP_R;HKD_R=G.HKD_R; A_LIM=G.A_LIM;H_LIM=G.H_LIM
L_HIP_PITCH,L_HIP_ROLL,L_ANKLE_PITCH,L_ANKLE_ROLL=0,1,4,5
R_HIP_PITCH,R_HIP_ROLL,R_ANKLE_PITCH,R_ANKLE_ROLL=6,7,10,11

FTH=40.0    # N Normalkraft -> Fuss belastet
GYR=0.8     # rad/s -> Koerper ruhig genug
RAMP=0.4

def foot_forces(m,d,lfb,rfb,floor_gid):
    fl=fr=0.0; f6=np.zeros(6)
    for i in range(d.ncon):
        c=d.contact[i]; g1,g2=c.geom1,c.geom2
        b1=m.geom_bodyid[g1]; b2=m.geom_bodyid[g2]
        is_floor = (g1==floor_gid or g2==floor_gid)
        if not is_floor: continue
        other_b = b2 if g1==floor_gid else b1
        mujoco.mj_contactForce(m,d,i,f6)
        fn=abs(f6[0])  # Normalkraft (Kontakt-Frame x = Normale)
        if other_b==lfb: fl+=fn
        elif other_b==rfb: fr+=fn
    return fl,fr

def run(t_rel, walk_v=0.6, min_settle=0.25, sim_dt=0.002, T=12.0):
    m=mujoco.MjModel.from_xml_path(A.SCENE); m.opt.timestep=sim_dt
    d=mujoco.MjData(m); mujoco.mj_resetData(m,d)
    wid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,"hold_base_weld")
    if wid>=0: m.eq_active0[wid]=0; d.eq_active[wid]=0
    lfb=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"left_ankle_roll_link")
    rfb=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,"right_ankle_roll_link")
    floor_gid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,"floor")
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
    def cl(x,l): return max(-l,min(l,x))
    state="RUN"; counter=0; released_t=None
    bal_entering=False; bal_t0=0.0; bal_q_start=None; ctrl_tau=np.zeros(12)
    handoff_t=None; xy_release=None; v_lurch=0.0
    nsteps=int(T/sim_dt)
    for s in range(nsteps):
        t=s*sim_dt
        if s % decim==0:
            quat=d.qpos[3:7]; omega=d.qvel[3:6]; g=grav(quat)
            walking = t<t_rel
            cmd=np.array([walk_v,0,0]) if walking else np.zeros(3)
            if released_t is None and not walking:
                released_t=t; xy_release=d.qpos[:2].copy()
            run_policy=(state=="RUN")
            # Doppelstuetz-Trigger nach Release
            if state=="RUN" and released_t is not None and (t-released_t)>min_settle:
                fl,fr=foot_forces(m,d,lfb,rfb,floor_gid)
                spin=np.linalg.norm(omega)
                if fl>FTH and fr>FTH and spin<GYR:
                    state="PD"; bal_entering=True; run_policy=False
            if run_policy:
                counter+=1; cmd_phys=cmd*max_cmd
                qj=(d.qpos[7:7+12]-default_angles)*dof_pos_scale; dqj=d.qvel[6:6+12]*dof_vel_scale
                count=counter*control_dt; phase=(count%gait_period)/gait_period
                obs[:3]=omega*ang_vel_scale; obs[3:6]=g; obs[6:9]=cmd_phys*cmd_scale
                obs[9:9+12]=qj; obs[9+12:9+24]=dqj; obs[9+24:9+36]=action
                obs[9+36]=math.sin(2*math.pi*phase); obs[9+36+1]=math.cos(2*math.pi*phase)
                with torch.no_grad():
                    action=policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
                target_dof[:12]=default_angles+action*action_scale
                for k in range(12): full_kp[k]=kps[k]; full_kd[k]=kds[k]
                ctrl_tau[:]=0.0
            else:
                if bal_entering:
                    bal_q_start=d.qpos[7:7+12].copy(); bal_t0=t; bal_entering=False; handoff_t=t
                ramp=1.0 if bal_q_start is None else max(0.0,min(1.0,(t-bal_t0)/RAMP))
                kp_eff=1.0+(BAL_KP-1.0)*ramp; kd_eff=math.sqrt(max(kp_eff,1e-6))
                pe,re=g[0],g[1]; pr,rr=omega[1],omega[0]
                tap=cl(AKP_P*pe+AKD_P*pr,A_LIM); tar=cl(-(AKP_R*re+AKD_R*rr),A_LIM)
                thp=cl(HKP_P*pe+HKD_P*pr,H_LIM); thr=cl(-(HKP_R*re+HKD_R*rr),H_LIM)
                ctrl_tau[:]=0
                ctrl_tau[L_ANKLE_PITCH]=ctrl_tau[R_ANKLE_PITCH]=tap; ctrl_tau[L_ANKLE_ROLL]=ctrl_tau[R_ANKLE_ROLL]=tar
                ctrl_tau[L_HIP_PITCH]=ctrl_tau[R_HIP_PITCH]=thp; ctrl_tau[L_HIP_ROLL]=ctrl_tau[R_HIP_ROLL]=thr
                ctrl_tau*=ramp
                qs=bal_q_start if bal_q_start is not None else default_angles
                target_dof[:12]=qs*(1.0-ramp)+default_angles*ramp
                for k in range(12): full_kp[k]=kps_pd[k]*kp_eff; full_kd[k]=kds_pd[k]*kd_eff
        q=d.qpos[7:7+nj]; dq=d.qvel[6:6+nj]
        tau=(target_dof-q)*full_kp+(0-dq)*full_kd; tau[:12]+=ctrl_tau
        d.ctrl[:]=tau; mujoco.mj_step(m,d)
        if handoff_t is not None and t>handoff_t: v_lurch=max(v_lurch,np.linalg.norm(d.qvel[:2]))
        if abs(grav(d.qpos[3:7])[0])>0.9 or d.qpos[2]<0.5:
            return dict(fell=True,t=t,t_rel=t_rel,handoff_t=handoff_t)
    drift=np.linalg.norm(d.qpos[:2]-xy_release) if xy_release is not None else 0.0
    return dict(fell=False,t_rel=t_rel,drift=drift,v_end=np.linalg.norm(d.qvel[:2]),
                v_lurch=v_lurch,handoff_t=handoff_t,
                handoff_delay=(handoff_t-t_rel) if handoff_t else None)

if __name__=="__main__":
    print("Doppelstuetz-Handoff: walk(0.6)->Release@t_rel->warte auf beide Fuesse belastet+ruhig->PD")
    print("Sweep t_rel (verschiedene Gangphasen). dt=0.002\n")
    print(f"{'t_rel[s]':<10}{'Ergebnis':<18}{'Handoff n.':<12}{'Lurch':<10}{'Drift':<10}{'v_end'}")
    print("-"*70)
    fell=0
    for t_rel in (4.0,4.1,4.2,4.3,4.4,4.5,4.6,4.7,4.8,4.9,5.0):
        r=run(t_rel)
        if r["fell"]:
            fell+=1; print(f"{t_rel:<10}{'GEFALLEN t=%.1f'%r['t']:<18}")
        else:
            hd="%.2fs"%r['handoff_delay'] if r['handoff_delay'] else "-"
            print(f"{t_rel:<10}{'steht':<18}{hd:<12}{r['v_lurch']:<10.3f}{r['drift']:<10.3f}{r['v_end']:.3f}")
    print(f"\n=> {11-fell}/11 Release-Zeitpunkte stehen sauber.")
