"""Testet sanften RUN->PD-Handoff: kurze Ausrollzeit + Posen-/Steifigkeits-Rampe.
Misst Sturz, Rueckwaerts-Ausschlag waehrend Ausrollen, und Lurch nach Handoff."""
import numpy as np, math, mujoco, torch
import audit_reference_deploy as A
policy=A.policy; recurrent=A.recurrent; da=A.default_angles; kps=A.kps; kds=A.kds
avs=A.ang_vel_scale; dps=A.dof_pos_scale; dvs=A.dof_vel_scale; ascl=A.action_scale
cs=A.cmd_scale; na=A.num_actions; no=A.num_obs; cdt=A.control_dt; gp=A.gait_period
grav=A.get_gravity_orientation; aw_idx=A.aw_idx; aw_kps=A.aw_kps; aw_kds=A.aw_kds; aw_t=A.aw_target
AKP_P=150;AKD_P=40;AKP_R=150;AKD_R=40;HKP_P=200;HKD_P=40;HKP_R=200;HKD_R=40;A_LIM=50;H_LIM=80
BAL_KP=10.0
LHP,LHR,LAP,LAR=0,1,4,5; RHP,RHR,RAP,RAR=6,7,10,11
def cl(x,l): return max(-l,min(l,x))

def run(sim_dt=0.001, settle=0.5, ramp_s=0.4, manual_immediate=False, T=12.0):
    m=mujoco.MjModel.from_xml_path(A.SCENE); m.opt.timestep=sim_dt
    d=mujoco.MjData(m); mujoco.mj_resetData(m,d)
    wid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,"hold_base_weld")
    if wid>=0: m.eq_active0[wid]=0; d.eq_active[wid]=0
    d.qpos[2]=0.78; d.qpos[3]=1; d.qpos[4]=d.qpos[5]=d.qpos[6]=0
    for i in range(m.nu): d.qpos[7+i]=da[i] if i<12 else 0.0
    d.qvel[:]=0; mujoco.mj_forward(m,d)
    if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
    nj=m.nu; decim=int(round(cdt/sim_dt)); action=np.zeros(na); obs=np.zeros(no,dtype=np.float32)
    tgt=np.zeros(nj)
    for j,idx in enumerate(aw_idx): tgt[idx]=aw_t[j]
    fkp=np.zeros(nj); fkd=np.zeros(nj)
    for j,idx in enumerate(aw_idx): fkp[idx]=aw_kps[j]; fkd[idx]=aw_kds[j]
    mode="PD"; cnt=0; x0=d.qpos[0]; stop_t=None; bal_t0=None; q_start=None
    ctrl_tau=np.zeros(12); x_min=0.0; v_after=0.0; t_walk=(2.0,6.0)
    for s in range(int(T/sim_dt)):
        t=s*sim_dt
        if s%decim==0:
            quat=d.qpos[3:7]; om=d.qvel[3:6]; g=grav(quat)
            if mode=="PD" and t_walk[0]<=t<t_walk[1] and stop_t is None:
                mode="POL"
                if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
                action[:]=0.0
            if mode=="POL" and stop_t is None and t>=t_walk[1]:
                stop_t=t
                if manual_immediate:  # sofort umschalten (wie START BALANCING im Lauf)
                    mode="BAL"; bal_t0=t; q_start=d.qpos[7:7+12].copy()
            if mode=="POL" and stop_t is not None and (t-stop_t)>settle:
                mode="BAL"; bal_t0=t; q_start=d.qpos[7:7+12].copy()
            if mode in ("PD","POL"):
                if mode=="POL":
                    cnt+=1
                    cmd=np.array([0.5,0,0.0]) if stop_t is None else np.zeros(3)
                    qj=(d.qpos[7:7+12]-da)*dps; dqj=d.qvel[6:6+12]*dvs
                    ph=((cnt*cdt)%gp)/gp
                    obs[:3]=om*avs; obs[3:6]=g; obs[6:9]=cmd*cs; obs[9:21]=qj; obs[21:33]=dqj
                    obs[33:45]=action; obs[45]=math.sin(2*math.pi*ph); obs[46]=math.cos(2*math.pi*ph)
                    with torch.no_grad(): action=policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
                    tgt[:12]=da+action*ascl
                    for k in range(12): fkp[k]=kps[k]; fkd[k]=kds[k]
                else:  # reiner PD-Stand (Phase 1)
                    pe,re=g[0],g[1]; pr,rr=om[1],om[0]
                    ctrl_tau[:]=0
                    ctrl_tau[LAP]=ctrl_tau[RAP]=cl(AKP_P*pe+AKD_P*pr,A_LIM); ctrl_tau[LAR]=ctrl_tau[RAR]=cl(-(AKP_R*re+AKD_R*rr),A_LIM)
                    ctrl_tau[LHP]=ctrl_tau[RHP]=cl(HKP_P*pe+HKD_P*pr,H_LIM); ctrl_tau[LHR]=ctrl_tau[RHR]=cl(-(HKP_R*re+HKD_R*rr),H_LIM)
                    tgt[:12]=da
                    for k in range(12): fkp[k]=kps[k]*BAL_KP; fkd[k]=kds[k]*math.sqrt(BAL_KP)
            else:  # BAL: sanfte Rampe von q_start->default, kp 1->BAL_KP, tau 0->voll
                pe,re=g[0],g[1]; pr,rr=om[1],om[0]
                r=cl((t-bal_t0)/ramp_s,1.0); r=max(0.0,r)
                kp_eff=1.0+(BAL_KP-1.0)*r
                ctrl_tau[:]=0
                ctrl_tau[LAP]=ctrl_tau[RAP]=cl(AKP_P*pe+AKD_P*pr,A_LIM)*r; ctrl_tau[LAR]=ctrl_tau[RAR]=cl(-(AKP_R*re+AKD_R*rr),A_LIM)*r
                ctrl_tau[LHP]=ctrl_tau[RHP]=cl(HKP_P*pe+HKD_P*pr,H_LIM)*r; ctrl_tau[LHR]=ctrl_tau[RHR]=cl(-(HKP_R*re+HKD_R*rr),H_LIM)*r
                tgt[:12]=q_start*(1-r)+da*r
                for k in range(12): fkp[k]=kps[k]*kp_eff; fkd[k]=kds[k]*math.sqrt(kp_eff)
        q=d.qpos[7:7+nj]; dq=d.qvel[6:6+nj]
        tau=(tgt-q)*fkp+(0-dq)*fkd
        if mode in ("PD","BAL"): tau[:12]+=ctrl_tau
        d.ctrl[:]=tau; mujoco.mj_step(m,d)
        if stop_t is not None:
            x_min=min(x_min, d.qpos[0]-x0)
            if t>stop_t+0.2: v_after=max(v_after, abs(d.qvel[0]))
        if abs(grav(d.qpos[3:7])[0])>0.9 or d.qpos[2]<0.5:
            return dict(fell=True,t=t,drift=d.qpos[0]-x0,x_min=x_min)
    return dict(fell=False,drift=d.qpos[0]-x0,x_min=x_min,v_after=v_after,vend=abs(d.qvel[0]))

print("Walk cmd=0.5 -> Release. Sanfte Rampe. x_min=max Rueckwaerts-Ausschlag ab Stop.\n")
print(f"{'Szenario':<34}{'Ergebnis':<22}{'drift':>8}{'rueckw.':>9}{'v_lurch':>9}")
print("-"*84)
for label,kw in [
    ("settle=0.5 ramp=0.4",       dict(settle=0.5,ramp_s=0.4)),
    ("settle=0.3 ramp=0.4",       dict(settle=0.3,ramp_s=0.4)),
    ("settle=0.6 ramp=0.5",       dict(settle=0.6,ramp_s=0.5)),
    ("settle=1.3 ramp=0.4 (alt)", dict(settle=1.3,ramp_s=0.4)),
    ("MANUAL sofort ramp=0.4",    dict(manual_immediate=True,ramp_s=0.4)),
    ("MANUAL sofort ramp=0.0(hart)",dict(manual_immediate=True,ramp_s=0.001)),
]:
    r=run(**kw)
    if r["fell"]: res=f"GEFALLEN t={r['t']:.1f}s"; extra=f"{r['drift']:>8.2f}{r['x_min']:>9.2f}{'-':>9}"
    else: res="steht am PD"; extra=f"{r['drift']:>8.2f}{r['x_min']:>9.2f}{r['v_after']:>9.2f}"
    print(f"{label:<34}{res:<22}{extra}")
