#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_leg_hold — Bring-up-Test fuer den Loco-Pfad OHNE RL-Policy.

Haelt Beine + Taille (Motor-Indizes 0..14) ueber rt/lowcmd in einer Standpose
(leicht gebeugte Knie) per PD. Damit laesst sich VOR dem RL-Controller pruefen:
  * Der Bridge-MERGE funktioniert (Beine aus rt/lowcmd, Arme weiter aus rt/arm_sdk),
  * die Basis ist frei (MuJoCo mit HOLD_BASE_MODE=off gestartet),
  * der Roboter laesst sich ueber die Beine grob aufrecht halten.

WICHTIG: MuJoCo muss mit freier Basis laufen:
    HOLD_BASE_MODE=off  (z.B. via run_sim_loco.sh)
Sonst haelt der Weld den Koerper ohnehin fest und der Test ist aussagelos.

Dies ist KEIN Balance-Regler (keine IMU-Rueckkopplung) — nur ein steifer Stand.
Ein langsames Kippen ist okay; es zeigt, dass die Beine aktuiert werden und die
Basis frei ist. Aktives Balancieren macht spaeter die RL-Policy (loco_sim).

Aufruf (im g1pilot-Container, nach source):
    ros2 run g1pilot sim_leg_hold
    ros2 run g1pilot sim_leg_hold --kp 200 --kd 5 --seconds 30
"""
import argparse
import math
import time
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1pilot.utils.common import resolve_dds, is_sim_mode

# Motor-Indizes Beine+Taille (0..14). Arme (15..28) ruehrt dieses Tool NICHT an.
LEG_WAIST = list(range(15))

# Standpose (rad) je Motorindex; nicht gelistete -> 0.0.
#  0 l_hip_pitch  3 l_knee  4 l_ankle_pitch | 6 r_hip_pitch 9 r_knee 10 r_ankle_pitch
STANCE = {0: -0.10, 3: 0.30, 4: -0.20, 6: -0.10, 9: 0.30, 10: -0.20}


def _log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] [sim_leg_hold] {msg}", flush=True)


def _roll_pitch_deg(quat):
    # Unitree imu_state.quaternion = [w, x, y, z] (aus MuJoCo framequat).
    w, x, y, z = quat[0], quat[1], quat[2], quat[3]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sp)
    return math.degrees(roll), math.degrees(pitch)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--interface", default="eth0", help="Real-Interface (im Sim wird 'lo' erzwungen)")
    p.add_argument("--kp", type=float, default=150.0)
    p.add_argument("--kd", type=float, default=4.0)
    p.add_argument("--rate", type=float, default=200.0, help="Sende-Rate [Hz]")
    p.add_argument("--ramp", type=float, default=1.5, help="kp-Anstiegszeit [s] (sanfter Start)")
    p.add_argument("--seconds", type=float, default=30.0)
    args = p.parse_args()

    domain, iface = resolve_dds(args.interface)
    _log(f"sim_mode={is_sim_mode()} -> DDS domain={domain}, iface={iface}")
    ChannelFactoryInitialize(domain, iface)

    latest = {"msg": None}

    def on_state(msg: LowState_):
        latest["msg"] = msg

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_state)

    _log("Warte auf rt/lowstate ...")
    t_wait = time.time()
    while latest["msg"] is None:
        if time.time() - t_wait > 5.0:
            _log("FEHLER: keine rt/lowstate. MuJoCo laeuft? richtige Domain?")
            return
        time.sleep(0.02)

    state = latest["msg"]
    _log(f"Verbunden. mode_machine={getattr(state, 'mode_machine', 'n/a')}")
    _log(f"Halte Beine+Taille auf Standpose, kp={args.kp:.0f} kd={args.kd:.0f} "
         f"(Ramp {args.ramp:.1f}s). ERINNERUNG: MuJoCo mit HOLD_BASE_MODE=off starten!")

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    crc = CRC()
    msg = unitree_hg_msg_dds__LowCmd_()
    msg.mode_pr = 0
    msg.mode_machine = int(getattr(state, "mode_machine", 0))

    dt = 1.0 / args.rate
    t0 = time.time()
    last_log = 0.0
    n = 0
    while time.time() - t0 < args.seconds:
        t = time.time() - t0
        ramp = min(1.0, t / args.ramp) if args.ramp > 0 else 1.0
        for i in LEG_WAIST:
            msg.motor_cmd[i].mode = 1
            msg.motor_cmd[i].kp = args.kp * ramp
            msg.motor_cmd[i].kd = args.kd
            msg.motor_cmd[i].q = STANCE.get(i, 0.0)
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].tau = 0.0
        msg.crc = crc.Crc(msg)
        pub.Write(msg)
        n += 1

        if t - last_log >= 0.5:
            last_log = t
            cur = latest["msg"]
            if cur is not None:
                lk = float(cur.motor_state[3].q)   # left knee
                rk = float(cur.motor_state[9].q)   # right knee
                roll, pitch = _roll_pitch_deg(cur.imu_state.quaternion)
                upright = "aufrecht" if abs(roll) < 20 and abs(pitch) < 20 else "KIPPT"
                _log(f"t={t:5.2f}s ramp={ramp:.2f} l_knee={lk:+.2f} r_knee={rk:+.2f} "
                     f"tilt(roll={roll:+5.1f} pitch={pitch:+5.1f})deg -> {upright}  (tx={n})")
        time.sleep(dt)

    _log("=" * 64)
    cur = latest["msg"]
    if cur is not None:
        roll, pitch = _roll_pitch_deg(cur.imu_state.quaternion)
        _log(f"ENDE: tilt roll={roll:+.1f} pitch={pitch:+.1f} deg.")
        if abs(roll) < 20 and abs(pitch) < 20:
            _log("-> Roboter steht noch grob aufrecht. Merge + freie Basis OK.")
        else:
            _log("-> Roboter ist gekippt. Beine WURDEN aktuiert (Merge OK), aber statischer "
                 "Stand reicht nicht — das aktive Balancieren macht die RL-Policy.")
    _log("Tipp: parallel Arme per Marker bewegen -> beides muss gleichzeitig laufen (Merge).")


if __name__ == "__main__":
    main()
