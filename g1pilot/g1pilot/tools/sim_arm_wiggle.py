#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_arm_wiggle — Standalone-Diagnose: bewegt EIN Arm-Gelenk in MuJoCo.

Testet die komplette Arm-Aktuierungskette (g1pilot -> rt/arm_sdk -> MuJoCo-Bridge
-> mj_data.ctrl) ohne ROS 2, IK oder Interactive Marker. Hält alle Arm-Gelenke
auf ihrer aktuellen Position und fährt ein einzelnes Zielgelenk sinusförmig.
Loggt Soll- vs. Ist-Winkel, damit klar erkennbar ist, ob MuJoCo dem Befehl folgt.

Aufruf (MuJoCo muss laufen, Arme dürfen frei sein — HOLD_BASE hält den Rumpf):
    G1_SIM_MODE=true python3 -m g1pilot.tools.sim_arm_wiggle
    G1_SIM_MODE=true ros2 run g1pilot sim_arm_wiggle -- --joint 22 --amp 0.4

Wichtige Joint-Indizes (DDS): 15-21 linker Arm, 22-28 rechter Arm.
  22 = right_shoulder_pitch (Default).
"""
import argparse
import math
import time
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import (
    resolve_dds, is_sim_mode,
    G1_29_JointArmIndex, G1_29_JointWristIndex, G1_29_JointIndex,
)


def _log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] [sim_arm_wiggle] {msg}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--joint", type=int, default=22,
                        help="DDS-Joint-Index (15-28), Default 22=right_shoulder_pitch")
    parser.add_argument("--amp", type=float, default=0.4, help="Amplitude [rad]")
    parser.add_argument("--period", type=float, default=4.0, help="Periode [s]")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--rate", type=float, default=200.0, help="Sende-Rate [Hz]")
    args = parser.parse_args()

    arm_vals = {m.value for m in G1_29_JointArmIndex}
    if args.joint not in arm_vals:
        _log(f"FEHLER: --joint {args.joint} ist kein Arm-Gelenk {sorted(arm_vals)}")
        return

    domain, iface = resolve_dds(args.interface)
    _log(f"sim_mode={is_sim_mode()}  ->  DDS domain={domain}, interface={iface}")
    ChannelFactoryInitialize(domain, iface)

    latest = {"msg": None}
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda m: latest.__setitem__("msg", m))

    _log("Warte auf erste LowState ...")
    t_wait = time.time()
    while latest["msg"] is None:
        if time.time() - t_wait > 10.0:
            _log("FEHLER: keine LowState in 10 s. Läuft MuJoCo / stimmt die Domain?")
            return
        time.sleep(0.02)

    state = latest["msg"]
    q0 = {i: float(state.motor_state[i].q) for i in arm_vals}
    center = q0[args.joint]
    _log(f"Verbunden. mode_machine={getattr(state, 'mode_machine', 'n/a')}")
    _log(f"Zielgelenk {args.joint}: Start q={center:.3f}, sweep ±{args.amp} rad")

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()
    crc = CRC()
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 1

    wrist_vals = {m.value for m in G1_29_JointWristIndex}
    # arm_sdk-Weight aktivieren (auf echtem Roboter nötig; in Sim ignoriert).
    try:
        msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
    except Exception:
        pass

    def gains(jid):
        return (40.0, 1.5) if jid in wrist_vals else (150.0, 4.0)

    # alle Arm-Gelenke auf Startposition halten
    for jid in arm_vals:
        kp, kd = gains(jid)
        msg.motor_cmd[jid].mode = 1
        msg.motor_cmd[jid].kp = kp
        msg.motor_cmd[jid].kd = kd
        msg.motor_cmd[jid].q = q0[jid]
        msg.motor_cmd[jid].dq = 0.0
        msg.motor_cmd[jid].tau = 0.0

    dt = 1.0 / args.rate
    t0 = time.time()
    last_log = 0.0
    n = 0
    while time.time() - t0 < args.seconds:
        t = time.time() - t0
        target = center + args.amp * math.sin(2.0 * math.pi * t / args.period)
        msg.motor_cmd[args.joint].q = target
        msg.crc = crc.Crc(msg)
        pub.Write(msg)
        n += 1

        if t - last_log >= 0.5:
            last_log = t
            cur = latest["msg"]
            meas = float(cur.motor_state[args.joint].q) if cur is not None else float("nan")
            err = target - meas
            _log(f"t={t:5.2f}s  cmd={target:+.3f}  meas={meas:+.3f}  err={err:+.3f}  (tx={n})")
        time.sleep(dt)

    # sanft auf Startposition zurück
    msg.motor_cmd[args.joint].q = center
    msg.crc = crc.Crc(msg)
    pub.Write(msg)
    cur = latest["msg"]
    meas = float(cur.motor_state[args.joint].q) if cur is not None else float("nan")
    moved = abs(meas - center)
    _log("=" * 60)
    _log(f"ERGEBNIS: gesendet={n} Befehle. Endwinkel meas={meas:+.3f} (Start {center:+.3f}).")
    if moved > 0.02 or n > 0:
        _log("Wenn sich das Gelenk in MuJoCo sichtbar bewegt hat und 'meas' dem 'cmd' "
             "folgt: Arm-Pfad OK. Sonst Logs schicken.")
    else:
        _log("Kein Tracking erkennbar — Logs + MuJoCo-Konsolenausgabe schicken.")


if __name__ == "__main__":
    main()
