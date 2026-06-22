#!/usr/bin/env python3
"""END-TO-END-VALIDIERUNG des Stepping-Stop (wie in loco_sim + Bridge implementiert).

Spiegelt EXAKT den ausgelieferten Pfad:
  Bridge: Fuss-Normalkraft + Basis-v (Welt) -> reserve[] (int-Round-Trip).
  loco_sim: reserve[] dekodieren -> per Stepping abbremsen (Gegen-Kommando ~ -k*v)
            -> bei |v|<vstop UND Doppelstuetz an PD-Stand (mit v-/yaw-Daempfung).

Misst ueber alle vier Kardinalrichtungen x mehrere Release-Zeitpunkte, ob der
Roboter sauber zum Stehen kommt. Erwartung (headless-Tuning): 16/16.
"""
import numpy as np, math, mujoco as mj, torch
import audit_reference_deploy as A, diag_gating as G, diag_contact_handoff as C
policy=A.policy; recurrent=A.recurrent; da=A.default_angles; kps=A.kps; kds=A.kds
aw_idx=A.aw_idx; aw_kps=A.aw_kps; aw_kds=A.aw_kds; aw_target=A.aw_target
def grav(q): return A.get_gravity_orientation(q)
def yaw_of(q):
    qw,qx,qy,qz=q; return math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
m=mj.MjModel.from_xml_path(A.SCENE); m.opt.timestep=0.002
lfb=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,"left_ankle_roll_link")
rfb=mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,"right_ankle_roll_link")
floor=mj.mj_name2id(m,mj.mjtObj.mjOBJ_GEOM,"floor")
# Parameter == loco_sim-Defaults
KB=2.5; BMAX=3.0; VSTOP=0.18; FTH=80.0; TMAX=2.5; HOLD=0.1
KV=120.0; YKD=25.0; HLIM=100.0; RAMP=0.4; KPSCALE=10.0
def reserve_roundtrip(fl,fr,vx,vy):
    r0=int(min(max(fl,0),4e6)); r1=int(min(max(fr,0),4e6))
    r2=int(round((min(max(vx,-10),10)+10)*1000)); r3=int(round((min(max(vy,-10),10)+10)*1000))
    if r0==0 and r1==0 and r2==0 and r3==0: return 0,0,0,0,False
    return float(r0),float(r1),r2/1000.0-10.0,r3/1000.0-10.0,True
def trial(vy,vyaw,walk_v,t_rel=4.5,T=13.0):
    d=mj.MjData(m); mj.mj_resetData(m,d)
    wid=mj.mj_name2id(m,mj.mjtObj.mjOBJ_EQUALITY,"hold_base_weld"); m.eq_active0[wid]=0; d.eq_active[wid]=0
    d.qpos[2]=0.78; d.qpos[3]=1
    for i in range(m.nu): d.qpos[7+i]=da[i] if i<12 else 0.0
    d.qvel[:]=0; mj.mj_forward(m,d)
    if recurrent: policy.hidden_state.zero_(); policy.cell_state.zero_()
    action=np.zeros(12); obs=np.zeros(47,dtype=np.float32); tgt=np.zeros(m.nu)
    for j,idx in enumerate(aw_idx): tgt[idx]=aw_target[j]
    fkp=np.zeros(m.nu); fkd=np.zeros(m.nu)
    for j,idx in enumerate(aw_idx): fkp[idx]=aw_kps[j]; fkd[idx]=aw_kds[j]
    counter=0; state="RUN"; released=None; braking=False; bt0=0; lowv0=None
    bal_t0=0; qcap=None
    def cl(x,l): return max(-l,min(l,x))
    for s in range(int(T/0.002)):
        t=s*0.002
        if s%10==0:
            quat=d.qpos[3:7]; om=d.qvel[3:6]; g=grav(quat); walking=t<t_rel
            counter+=1
            fl_t,fr_t=C.foot_forces(m,d,lfb,rfb,floor)
            fl,fr,vxw,vyw,valid=reserve_roundtrip(fl_t,fr_t,d.qvel[0],d.qvel[1])  # Bridge<->loco_sim
            yaw=yaw_of(quat); cy,sy=math.cos(yaw),math.sin(yaw)
            vxb=cy*vxw+sy*vyw; vyb=-sy*vxw+cy*vyw; vmag=math.hypot(vxb,vyb)
            if state=="RUN":
                if walking and not braking:
                    cmd=np.array([walk_v,vy,vyaw])
                else:
                    if released is None: released=t
                    if not braking: braking=True; bt0=t; lowv0=None
                    bx=cl(-KB*vxb/A.max_cmd[0],BMAX); by=cl(-KB*vyb/A.max_cmd[1],BMAX)
                    cmd=np.array([bx,by,0.0])
                    vth=VSTOP if (t-bt0)<TMAX else VSTOP+0.12
                    if vmag<vth and fl>FTH and fr>FTH:
                        if lowv0 is None: lowv0=t
                        elif t-lowv0>HOLD: state="PD"; qcap=d.qpos[7:19].copy(); bal_t0=t
                    else: lowv0=None
                cp=cmd*A.max_cmd; ph=(counter*0.02%A.gait_period)/A.gait_period
                obs[:3]=om*A.ang_vel_scale; obs[3:6]=g; obs[6:9]=cp*A.cmd_scale
                obs[9:21]=(d.qpos[7:19]-da)*A.dof_pos_scale; obs[21:33]=d.qvel[6:18]*A.dof_vel_scale
                obs[33:45]=action; obs[45]=math.sin(2*math.pi*ph); obs[46]=math.cos(2*math.pi*ph)
                with torch.no_grad(): action=policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
                tgt[:12]=da+action*A.action_scale
                for k in range(12): fkp[k]=kps[k]; fkd[k]=kds[k]
                ctau=np.zeros(12)
            else:
                ramp=min(1.0,(t-bal_t0)/RAMP); kpe=1+(KPSCALE-1)*ramp; kde=math.sqrt(kpe)
                pe,re=g[0],g[1]; pr,rr=om[1],om[0]; yr=om[2]
                ctau=np.zeros(12)
                ctau[4]=ctau[10]=cl(150*pe+40*pr+KV*vxb,50); ctau[5]=ctau[11]=cl(-(150*re+40*rr+KV*vyb),50)
                ctau[0]=ctau[6]=cl(200*pe+40*pr,HLIM); ctau[1]=ctau[7]=cl(-(200*re+40*rr),HLIM)
                ctau[2]=ctau[8]=cl(-YKD*yr,40)
                ctau*=ramp; tgt[:12]=qcap*(1-ramp)+da*ramp
                for k in range(12): fkp[k]=G.kps_pd[k]*kpe; fkd[k]=G.kds_pd[k]*kde
        q=d.qpos[7:7+m.nu]; dq=d.qvel[6:6+m.nu]
        tau=(tgt-q)*fkp-dq*fkd; tau[:12]+=ctau; d.ctrl[:]=tau; mj.mj_step(m,d)
        gg=grav(d.qpos[3:7])
        if abs(gg[0])>0.9 or abs(gg[1])>0.9 or d.qpos[2]<0.5: return "FIEL"
    return "steht" if state=="PD" else "laeuft"
if __name__=="__main__":
    print("End-to-End Stepping-Stop (Bridge reserve[] round-trip + loco_sim-Logik):\n")
    dirs=[("vorwaerts",0,0,0.6),("seitwaerts-L",0.5,0,0),("seitwaerts-R",-0.5,0,0),("rueckwaerts",0,0,-0.4)]
    tot=ok=0
    for nm,vy,vyaw,wv in dirs:
        rs=[trial(vy,vyaw,wv,t_rel=tr) for tr in (4.35,4.5,4.65,4.8)]
        o=sum(r=="steht" for r in rs); tot+=len(rs); ok+=o
        print(f"  {nm:<14} {o}/4 steht   {rs}")
    print(f"\n=> {ok}/{tot} sauber gestoppt.")
