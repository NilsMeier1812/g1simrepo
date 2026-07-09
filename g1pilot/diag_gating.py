#!/usr/bin/env python3
"""DIAGNOSE: Warum macht das Command-Gating (054d425) das Laufen INSTABIL?

Reproduziert headless das Verhalten, das der Nutzer in der Live-Sim sieht:
  * a7d08c5 (CONTINUOUS): Policy laeuft im RUN DURCHGEHEND -> Joystick in alle
    Richtungen ziehen ist stabil; nur bei cmd=0 driftet sie (policy-inhaerent).
  * 054d425 (GATED): Policy laeuft NUR bei cmd>eps; sinkt das Kommando unter eps
    (z.B. Stick durch die Mitte), wird der LSTM-Zustand genullt UND ein steifer
    PD schnappt rein. Hypothese: Genau das macht das Laufen instabil.

Es wird EIN realistisches Joystick-Profil gefahren (vorwaerts, kurz durch die
Mitte, wieder vorwaerts, seitwaerts, dann loslassen) und fuer beide Strategien
gemessen, ob/wann der Roboter faellt und wie weit er driftet.

Saubere qpos/qvel-Obs-Pipeline (== unitree_rl_gym; Sensoren==qpos bereits
verifiziert). Nutzt die Policy/Config aus audit_reference_deploy.
"""
import numpy as np, math, mujoco, torch
import audit_reference_deploy as A

cfg=A.cfg; policy=A.policy; recurrent=A.recurrent
default_angles=A.default_angles; kps=A.kps; kds=A.kds
aw_idx=A.aw_idx; aw_kps=A.aw_kps; aw_kds=A.aw_kds; aw_target=A.aw_target
ang_vel_scale=A.ang_vel_scale; dof_pos_scale=A.dof_pos_scale; dof_vel_scale=A.dof_vel_scale
action_scale=A.action_scale; cmd_scale=A.cmd_scale; num_actions=A.num_actions; num_obs=A.num_obs
control_dt=A.control_dt; gait_period=A.gait_period; max_cmd=A.max_cmd
grav=A.get_gravity_orientation

# PD-Balancer-Gains (identisch zu loco_sim._send_balance_pd)
BAL_KP=10.0; AKP_P=150; AKD_P=40; AKP_R=150; AKD_R=40; HKP_P=200; HKD_P=40; HKP_R=200; HKD_R=40
A_LIM=50; H_LIM=80
kps_pd=np.array([100,100,100,150,40,40,100,100,100,150,40,40],dtype=float)
kds_pd=np.array([2,2,2,4,2,2,2,2,2,4,2,2],dtype=float)
L_HIP_PITCH,L_HIP_ROLL,L_ANKLE_PITCH,L_ANKLE_ROLL=0,1,4,5
R_HIP_PITCH,R_HIP_ROLL,R_ANKLE_PITCH,R_ANKLE_ROLL=6,7,10,11
EPS=0.06
WALK_SETTLE=0.5
BAL_RAMP=0.4


def joystick(t):
    """Realistisches Stick-Profil (normiert [-1,1]). Nach WALK wird SOFORT gefahren
    (wie der Nutzer es macht), ab 8s losgelassen (=0). Mehrere DIPS durch die Mitte
    (<eps) -> Gating-Trigger (Stick kreuzt Zentrum / wird kurz losgelassen)."""
    if t<0.5:  return np.array([0.4,0,0])      # sofort vorwaerts anlaufen
    if t<2.0:  return np.array([0.6,0,0])      # vorwaerts
    if t<2.25: return np.array([0.0,0,0])      # DIP durch Mitte
    if t<3.5:  return np.array([0.7,0,0])      # wieder vorwaerts
    if t<3.75: return np.array([0.02,0,0])     # DIP (knapp unter eps)
    if t<5.0:  return np.array([0.0,0.5,0])    # seitwaerts links
    if t<5.25: return np.array([0.0,0,0])      # DIP
    if t<8.0:  return np.array([0.5,0,0.3])    # vorwaerts + drehen
    return np.array([0.0,0,0])                 # losgelassen


def run(strategy, sim_dt=0.002, T=12.0, verbose=False):
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
    def cl(x,l): return max(-l,min(l,x))

    # FSM-Status wie loco_sim
    state="RUN"; walk_active=False; counter=0; zero_t0=None
    bal_entering=False; bal_t0=0.0; bal_q_start=None
    counter=0; start_xy=d.qpos[:2].copy(); ctrl_tau=np.zeros(12)
    max_tilt=0.0; v_peak=0.0; handoff_t=None
    nsteps=int(T/sim_dt)
    for s in range(nsteps):
        t=s*sim_dt
        if s % decim==0:
            quat=d.qpos[3:7]; omega=d.qvel[3:6]; g=grav(quat)
            tilt=max(abs(g[0]),abs(g[1])); max_tilt=max(max_tilt,tilt)
            cmd=joystick(t); cmd_mag=float(np.linalg.norm(cmd))

            run_policy=False
            if strategy=="continuous":
                # a7d08c5: im RUN laeuft die Policy IMMER; nach settle bei cmd~0 -> PD
                if state=="RUN":
                    if cmd_mag>EPS: zero_t0=None
                    else:
                        if zero_t0 is None: zero_t0=t
                        elif t-zero_t0>WALK_SETTLE:
                            state="PD"; bal_entering=True
                    run_policy=(state=="RUN")
            elif strategy=="gated":
                # 054d425: Policy NUR bei cmd>eps; sonst PD + LSTM-Reset bei Reaktivierung
                if state=="RUN":
                    if cmd_mag>EPS:
                        if not walk_active:
                            if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
                            action[:]=0.0; counter=0; walk_active=True
                        zero_t0=None; run_policy=True
                    else:
                        if not walk_active and zero_t0 is None: zero_t0=t
                        run_policy=False  # PD haelt

            if run_policy:
                counter+=1
                cmd_phys=cmd*max_cmd
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
                # PD-Balancer (mit sanfter Eintritts-Rampe wie loco_sim)
                if bal_entering:
                    bal_q_start=d.qpos[7:7+12].copy(); bal_t0=t; bal_entering=False
                    if handoff_t is None: handoff_t=t
                ramp=1.0 if bal_q_start is None else max(0.0,min(1.0,(t-bal_t0)/BAL_RAMP))
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
        # PD jeden Sim-Schritt
        q=d.qpos[7:7+nj]; dq=d.qvel[6:6+nj]
        tau=(target_dof-q)*full_kp+(0-dq)*full_kd
        tau[:12]+=ctrl_tau
        d.ctrl[:]=tau
        mujoco.mj_step(m,d)
        v_peak=max(v_peak,np.linalg.norm(d.qvel[:2]))
        if abs(grav(d.qpos[3:7])[0])>0.9 or d.qpos[2]<0.5:
            return dict(fell=True, t=t, drift=np.linalg.norm(d.qpos[:2]-start_xy),
                        state=state, handoff_t=handoff_t, max_tilt=max_tilt)
    return dict(fell=False, drift=np.linalg.norm(d.qpos[:2]-start_xy),
                v_end=np.linalg.norm(d.qvel[:2]), state=state, handoff_t=handoff_t,
                max_tilt=max_tilt, v_peak=v_peak)


if __name__=="__main__":
    print("Joystick-Profil: steht(0-2) walk(0.6) DIP walk(0.7) DIP seit(0.5) DIP walk+yaw los(8+)")
    print("Frage: macht das Command-Gating (LSTM-Reset bei jedem cmd<eps) das Laufen instabil?\n")
    print(f"{'Strategie':<16}{'Ergebnis':<30}{'Details'}")
    print("-"*78)
    for sim_dt in (0.002, 0.001):
        for strat in ("continuous","gated"):
            r=run(strat, sim_dt=sim_dt)
            tag=f"{strat} dt={sim_dt}"
            if r["fell"]:
                res=f"GEFALLEN t={r['t']:.1f}s"
                det=f"drift={r['drift']:.2f}m tilt_max={r['max_tilt']:.2f} state={r['state']}"
            else:
                res="steht durch"
                det=f"drift={r['drift']:.2f}m v_end={r['v_end']:.2f} tilt_max={r['max_tilt']:.2f}"
            print(f"{tag:<16}{res:<30}{det}")
