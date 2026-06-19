#!/usr/bin/env python3
"""DAS Kern-Szenario: Stehen (PD) -> Gehen (Policy) -> Anhalten -> Stehen (PD).

Testet den kritischen Uebergang Gehen->Stehen: kann die Policy beim Anhalten die
Geschwindigkeit so weit nullen, dass der PD-Balancer sauber uebernimmt und steht?

Ablauf je Lauf:
  Phase 1 (0-2s)   PD-Stand.
  Phase 2 (2-5s)   Policy WARM, cmd=[vx,0,0] -> vorwaerts gehen.
  Phase 3 (ab 5s)  ANHALTEN per gewaehlter Strategie; sobald |v|<v_hand fuer
                   hand_hold s -> zurueck zu PD.
  Phase 4          PD haelt (oder faellt). Misst Endgeschwindigkeit + ob er steht.

Strategien:
  hard0  : cmd sofort 0.
  ramp   : cmd rampt vx->0 ueber ramp_s, dann 0.
  velfb  : cmd = -kv * Geschw. (im Blickrichtungs-KS), aktiv bremsen.
  rampvf : erst Rampe, dann velfb (kombiniert).
"""
import numpy as np, math, mujoco, torch
import experiment_push_recovery as E, experiment_stop_after_push as S

WALK_V = 0.4            # normierter Vorwaerts-cmd in Phase 2
T1, T2 = 2.0, 5.0       # Ende Phase1 / Ende Phase2 (Stop-Beginn)
T = 12.0
sim_dt=E.sim_dt; dec=E.decimation; cdt=E.control_dt


def yaw_of(q):
    w,x,y,z=q; return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def run(strategy, kv=1.2, ramp_s=1.0, v_hand=0.18, hand_hold=0.4, ema=0.25):
    m=mujoco.MjModel.from_xml_path(E.SCENE); m.opt.timestep=sim_dt
    d=mujoco.MjData(m); E.reset_stance(m,d)
    if E.recurrent: E.policy.hidden_state.zero_(); E.policy.cell_state.zero_()
    n=m.nu; action=np.zeros(E.na); counter=0; kd_scale=math.sqrt(E.BAL_KP)
    p_prev=d.qpos[:2].copy(); vsm=np.zeros(2)
    mode="PD"; stop_t=None; calm0=None; walk_xy=None
    def cl(x,l): return max(-l,min(l,x))
    vmax_after=0.0
    for tick in range(int(T/cdt)):
        t=tick*cdt; sd=d.sensordata
        quat=sd[3*n:3*n+4]; gyro=sd[3*n+4:3*n+7].copy(); g=E.grav(quat); tilt=max(abs(g[0]),abs(g[1]))
        vel=(d.qpos[:2]-p_prev)/cdt; p_prev=d.qpos[:2].copy(); vsm=(1-ema)*vsm+ema*vel; sp=np.linalg.norm(vsm)
        # Phasen-Steuerung
        if mode=="PD" and t>=T1 and stop_t is None and t<T2:
            mode="POL"   # losgehen
            if E.recurrent: E.policy.hidden_state.zero_(); E.policy.cell_state.zero_()
            action[:]=0.0
        if mode=="POL" and stop_t is None and t>=T2:
            stop_t=t; walk_xy=d.qpos[:2].copy()
        if mode=="POL" and stop_t is not None:
            if sp<v_hand and tilt<0.12:
                if calm0 is None: calm0=t
                elif t-calm0>hand_hold: mode="PD"
            else: calm0=None
        if stop_t is not None and t>stop_t+0.3: vmax_after=max(vmax_after,sp)
        # Kommando
        cmd_q=np.zeros(n);cmd_kp=np.zeros(n);cmd_kd=np.zeros(n);cmd_tau=np.zeros(n)
        if mode=="PD":
            pe,re=g[0],g[1];pr,rr=gyro[1],gyro[0]
            tap=cl(E.AKP_P*pe+E.AKD_P*pr,E.A_LIM); tar=cl(-(E.AKP_R*re+E.AKD_R*rr),E.A_LIM)
            thp=cl(E.HKP_P*pe+E.HKD_P*pr,E.H_LIM); thr=cl(-(E.HKP_R*re+E.HKD_R*rr),E.H_LIM)
            tau=np.zeros(12); tau[E.L_ANKLE_PITCH]=tau[E.R_ANKLE_PITCH]=tap; tau[E.L_ANKLE_ROLL]=tau[E.R_ANKLE_ROLL]=tar
            tau[E.L_HIP_PITCH]=tau[E.R_HIP_PITCH]=thp; tau[E.L_HIP_ROLL]=tau[E.R_HIP_ROLL]=thr
            for k in range(12): cmd_q[k]=E.default_angles[k];cmd_tau[k]=tau[k];cmd_kp[k]=E.kps_pd[k]*E.BAL_KP;cmd_kd[k]=E.kds_pd[k]*kd_scale
            for j,idx in enumerate(E.aw_idx):
                sc=E.BAL_KP if idx in (12,13,14) else 1.0
                cmd_q[idx]=0;cmd_kp[idx]=E.aw_kps[j]*sc;cmd_kd[idx]=E.aw_kds[j]*math.sqrt(sc)
        else:
            counter+=1
            if stop_t is None:
                cmd=np.array([WALK_V,0,0],dtype=float)     # Phase 2: gehen
            else:
                el=t-stop_t
                yw=yaw_of(quat); c,s=math.cos(yw),math.sin(yw)
                vh=np.array([c*vsm[0]+s*vsm[1], -s*vsm[0]+c*vsm[1]])  # Geschw. im Blickrichtungs-KS
                if strategy=="hard0": cmd=np.zeros(3)
                elif strategy=="ramp":
                    f=max(0.0,1-el/ramp_s); cmd=np.array([WALK_V*f,0,0])
                elif strategy=="velfb":
                    cmd=np.array([cl(-kv*vh[0],1.0),cl(-kv*vh[1],1.0),0.0])
                elif strategy=="rampvf":
                    if el<ramp_s: f=1-el/ramp_s; cmd=np.array([WALK_V*f,0,0])
                    else: cmd=np.array([cl(-kv*vh[0],1.0),cl(-kv*vh[1],1.0),0.0])
            tgt,action=S.policy_targets(d,m,n,action,cmd,counter)
            for k,idx in enumerate(E.leg_idx): cmd_q[idx]=tgt[k];cmd_kp[idx]=E.kps_p[k];cmd_kd[idx]=E.kds_p[k]
            for j,idx in enumerate(E.awi): cmd_q[idx]=E.awt[j];cmd_kp[idx]=E.awk[j];cmd_kd[idx]=E.awd[j]
        for _ in range(dec):
            sd=d.sensordata
            for i in range(n): d.ctrl[i]=cmd_tau[i]+cmd_kp[i]*(cmd_q[i]-sd[i])+cmd_kd[i]*(0-sd[i+n])
            mujoco.mj_step(m,d)
        if E.fallen(m,d,n):
            return dict(fell=True, t_fall=t, mode=mode)
    # Stand-Qualitaet in den letzten 2 s messen
    end_xy=d.qpos[:2].copy()
    return dict(fell=False, mode=mode, vmax_after=vmax_after, v_end=sp,
                handed=(mode=="PD"),
                creep_after_hand=np.nan)


if __name__=="__main__":
    print(f"Gehen (cmd={WALK_V}) -> Anhalten -> PD. v_hand=0.18, dann Uebergabe an PD.\n")
    print(f"{'Stop-Strategie':<22}{'Ergebnis':<12}{'an PD uebergeben':>17}{'v_max n.Stop':>13}{'v_end':>8}")
    print('-'*72)
    cfgs=[("hard0",dict(strategy='hard0')),
          ("ramp 1.0s",dict(strategy='ramp',ramp_s=1.0)),
          ("velfb kv=1.2",dict(strategy='velfb',kv=1.2)),
          ("velfb kv=2.0",dict(strategy='velfb',kv=2.0)),
          ("rampvf kv=1.2",dict(strategy='rampvf',kv=1.2,ramp_s=0.8)),
          ("rampvf kv=2.0",dict(strategy='rampvf',kv=2.0,ramp_s=0.8))]
    for label,kw in cfgs:
        r=run(**kw)
        if r["fell"]:
            tf=f"t={r['t_fall']:.1f}s"
            print(f"{label:<22}{'GEFALLEN':<12}{'-':>17}{'-':>13}{tf:>8}")
        else:
            print(f"{label:<22}{('steht' if r['mode']=='PD' else 'laeuft noch'):<12}"
                  f"{('JA' if r['handed'] else 'nein'):>17}{r['vmax_after']:>13.2f}{r['v_end']:>8.2f}")
