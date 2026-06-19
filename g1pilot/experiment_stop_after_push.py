#!/usr/bin/env python3
"""Warum driftet die RL-Policy nach einem Stoss endlos, und wie stoppt man sie?

Die Policy-Obs (47) enthalten KEINE lineare Geschwindigkeit -> bei aufrechtem,
nicht drehendem Koerper ist die Policy blind fuer ihre horizontale Drift und kann
sie bei cmd=0 nicht nullen. Hebel: der cmd-Eingang. Dieses Experiment vergleicht:

  A cmd0    : reine Policy, cmd=0            (Baseline: driftet endlos)
  B velfb   : cmd = -gain * Drift-Geschw.    (Geschwindigkeit aktiv nullen)
  C velpos  : cmd = -kv*v - kp*pos           (zusaetzlich zur Startposition zurueck)
  D timebox : Policy nur kurz, dann zurueck zu PD (Fuesse am Boden bremsen Restdrift)

Stoss: 150 N, 120 ms, vorwaerts, bei t=2 s. Misst Restdrift + Endgeschwindigkeit.
"""
import numpy as np, math, mujoco, torch
import experiment_push_recovery as E

PUSH_F = 150.0; PUSH_DIR = "vorne"; T = 9.0
sim_dt = E.sim_dt; dec = E.decimation; ctrl_dt = E.control_dt


def policy_targets(d, m, n, action, cmd, counter):
    qj = np.array([d.sensordata[i] for i in E.leg_idx])
    dqj = np.array([d.sensordata[i+n] for i in E.leg_idx])
    quat = d.sensordata[3*n:3*n+4]; av = d.sensordata[3*n+4:3*n+7].copy()
    g = E.grav(quat); phase = (counter*ctrl_dt % E.gait)/E.gait
    obs = np.zeros(E.no)
    obs[:3]=av*E.avs; obs[3:6]=g; obs[6:9]=cmd*E.cmd_scale*E.max_cmd
    obs[9:9+E.na]=(qj-E.da)*E.dps; obs[9+E.na:9+2*E.na]=dqj*E.dvs; obs[9+2*E.na:9+3*E.na]=action
    obs[9+3*E.na]=math.sin(2*math.pi*phase); obs[9+3*E.na+1]=math.cos(2*math.pi*phase)
    with torch.no_grad():
        action = E.policy(torch.from_numpy(obs.astype(np.float32)).unsqueeze(0)).numpy().squeeze()
    return E.da + action*E.ascl, action


def run(strategy, kv=0.0, kp=0.0, timebox_s=0.8, grace_s=0.35, ema=0.25):
    m = mujoco.MjModel.from_xml_path(E.SCENE); m.opt.timestep=sim_dt
    d = mujoco.MjData(m); E.reset_stance(m, d)
    if E.recurrent: E.policy.hidden_state.zero_(); E.policy.cell_state.zero_()
    n=m.nu; tb=m.body("torso_link").id
    action=np.zeros(E.na); cmd_q=np.zeros(n); cmd_kp=np.zeros(n); cmd_kd=np.zeros(n); cmd_tau=np.zeros(n)
    kd_scale=math.sqrt(E.BAL_KP)
    nticks=int(T/ctrl_dt); counter=0
    mode="PD"; t_catch=None; pv=E.push_vec(PUSH_F, PUSH_DIR)
    p_prev=d.qpos[:2].copy(); start_xy=d.qpos[:2].copy()
    vel=np.zeros(2); vsm=np.zeros(2)   # vsm = EMA-geglaettete Drift-Geschwindigkeit
    speed_hist=[]; calm_t0=None
    def clamp(x,l): return max(-l,min(l,x))
    for tick in range(nticks):
        t=tick*ctrl_dt
        quat=d.sensordata[3*n:3*n+4]; gyro=d.sensordata[3*n+4:3*n+7].copy(); g=E.grav(quat)
        tilt=max(abs(g[0]),abs(g[1]))
        vel = (d.qpos[:2]-p_prev)/ctrl_dt; p_prev=d.qpos[:2].copy()
        vsm = (1-ema)*vsm + ema*vel
        sp = np.linalg.norm(vsm)
        # Umschalt-Logik PD->Policy bei Stoss
        if mode=="PD" and tilt>0.15:
            mode="POL"; t_catch=t; calm_t0=None
            if E.recurrent: E.policy.hidden_state.zero_(); E.policy.cell_state.zero_()
            action[:]=0.0
        # Rueckkehr Policy->PD je nach Strategie
        if mode=="POL":
            if strategy=="timebox" and t-t_catch>timebox_s:
                mode="PD"
            if strategy in ("velfb","velpos"):
                calm = sp<0.12 and tilt<0.10
                if calm:
                    if calm_t0 is None: calm_t0=t
                    elif t-calm_t0>0.5: mode="PD"
                else: calm_t0=None
        # Kommando
        if mode=="PD":
            pe,re=g[0],g[1]; pr,rr=gyro[1],gyro[0]
            t_ap=clamp(E.AKP_P*pe+E.AKD_P*pr,E.A_LIM); t_ar=clamp(-(E.AKP_R*re+E.AKD_R*rr),E.A_LIM)
            t_hp=clamp(E.HKP_P*pe+E.HKD_P*pr,E.H_LIM); t_hr=clamp(-(E.HKP_R*re+E.HKD_R*rr),E.H_LIM)
            tau=np.zeros(12)
            tau[E.L_ANKLE_PITCH]=tau[E.R_ANKLE_PITCH]=t_ap; tau[E.L_ANKLE_ROLL]=tau[E.R_ANKLE_ROLL]=t_ar
            tau[E.L_HIP_PITCH]=tau[E.R_HIP_PITCH]=t_hp; tau[E.L_HIP_ROLL]=tau[E.R_HIP_ROLL]=t_hr
            for k in range(12):
                cmd_q[k]=E.default_angles[k]; cmd_tau[k]=tau[k]
                cmd_kp[k]=E.kps_pd[k]*E.BAL_KP; cmd_kd[k]=E.kds_pd[k]*kd_scale
            for j,idx in enumerate(E.aw_idx):
                sc=E.BAL_KP if idx in (12,13,14) else 1.0
                cmd_q[idx]=0.0; cmd_tau[idx]=0.0; cmd_kp[idx]=E.aw_kps[j]*sc; cmd_kd[idx]=E.aw_kds[j]*math.sqrt(sc)
        else:
            counter+=1
            in_grace = (t-t_catch) < grace_s   # erst natuerlichen Fangschritt zulassen
            if strategy=="cmd0" or in_grace: cmd=np.zeros(3)
            elif strategy=="velfb":
                cmd=np.array([clamp(-kv*vsm[0],1.0), clamp(-kv*vsm[1],1.0), 0.0])
            elif strategy=="velpos":
                dpos=d.qpos[:2]-start_xy
                cmd=np.array([clamp(-kv*vsm[0]-kp*dpos[0],1.0), clamp(-kv*vsm[1]-kp*dpos[1],1.0), 0.0])
            elif strategy=="timebox": cmd=np.zeros(3)
            target,action=policy_targets(d,m,n,action,cmd,counter)
            for k,idx in enumerate(E.leg_idx): cmd_q[idx]=target[k]; cmd_kp[idx]=E.kps_p[k]; cmd_kd[idx]=E.kds_p[k]; cmd_tau[idx]=0.0
            for j,idx in enumerate(E.awi): cmd_q[idx]=E.awt[j]; cmd_kp[idx]=E.awk[j]; cmd_kd[idx]=E.awd[j]; cmd_tau[idx]=0.0
        for _ in range(dec):
            d.xfrc_applied[tb] = pv if (E.PUSH_T0<=t<E.PUSH_T0+E.PUSH_DUR) else 0.0
            sd=d.sensordata
            for i in range(n): d.ctrl[i]=cmd_tau[i]+cmd_kp[i]*(cmd_q[i]-sd[i])+cmd_kd[i]*(0.0-sd[i+n])
            mujoco.mj_step(m,d)
        if t>E.PUSH_T0: speed_hist.append(sp)
        if E.fallen(m,d,n):
            drift=np.linalg.norm(d.qpos[:2]-start_xy)
            return dict(fell=True, drift=drift, vend=sp, mode=mode, t_fall=t)
    drift=np.linalg.norm(d.qpos[:2]-start_xy)
    return dict(fell=False, drift=drift, vend=sp, mode=mode,
                vmax=max(speed_hist) if speed_hist else 0.0)


if __name__=="__main__":
    print(f"Stoss {PUSH_F:.0f} N {PUSH_DIR}, {T:.0f}s. Restdrift = Endabstand vom Start.\n")
    print(f"{'Strategie':<26}{'gefallen':>9}{'Restdrift[m]':>13}{'v_end[m/s]':>11}{'End-Mode':>10}")
    print("-"*79)
    cfgs = [
        ("A cmd0 (Baseline)",           dict(strategy="cmd0")),
        ("B velfb kv=0.5 +glatt",       dict(strategy="velfb", kv=0.5)),
        ("B velfb kv=1.0 +glatt",       dict(strategy="velfb", kv=1.0)),
        ("B velfb kv=2.0 +glatt",       dict(strategy="velfb", kv=2.0)),
        ("C velpos kv=1.0 kp=0.5",      dict(strategy="velpos", kv=1.0, kp=0.5)),
        ("C velpos kv=1.5 kp=0.8",      dict(strategy="velpos", kv=1.5, kp=0.8)),
    ]
    for label,kw in cfgs:
        r=run(**kw)
        fell = "JA" if r["fell"] else "nein"
        print(f"{label:<26}{fell:>9}{r['drift']:>13.2f}{r['vend']:>11.2f}{r['mode']:>10}")
