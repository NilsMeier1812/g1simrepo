#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_arm_monitor — schneidet mit, was der arm_controller an MuJoCo SENDET.

Hoert beide Unitree-DDS-Topics ab (kein ROS noetig):
  rt/arm_sdk  (LowCmd_)   = Sollwerte, die der arm_controller publiziert
  rt/lowstate (LowState_) = Istwerte, die MuJoCo zurueckmeldet

Damit laesst sich das "wilde Zappeln" eindeutig einordnen:
  * Oszilliert das KOMMANDO (cmd-q springt gross/schnell)        -> Controller/IK-Bug
  * Kommando glatt, aber Ist-q weicht stark ab / divergiert      -> Gains/Physik
  * Wird nur EIN Arm oder werden BEIDE Arme kommandiert?         -> zeigt Spalten
  * arm_sdk-Weight (Index 29) korrekt?                           -> wird geloggt

Pro Fenster (Default 0.5 s) wird je Arm geloggt:
  cmd  = zuletzt gesendeter Soll-q
  meas = zuletzt gemessener Ist-q
  d    = |cmd - meas| (Schleppfehler; gross = PD kaempft)
  osc  = Spannweite (max-min) des Soll-q im Fenster (gross+schnell = Kommando schwingt)

Aufruf (MuJoCo + g1pilot laufen, Arme aktiviert, Ziel gesendet/Marker gezogen):
    ros2 run g1pilot sim_arm_monitor
    ros2 run g1pilot sim_arm_monitor --window 0.25 --seconds 30
"""
import argparse
import time
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from g1pilot.utils.common import resolve_dds, is_sim_mode
from g1pilot.utils.joints_names import RIGHT_JOINT_INDICES_LIST, LEFT_JOINT_INDICES_LIST

WEIGHT_IDX = 29  # kNotUsedJoint0 -> arm_sdk Weight


def _log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] [arm_monitor] {msg}", flush=True)


def _fmt(vals):
    return "[" + " ".join(f"{v:+.2f}" for v in vals) + "]"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="eth0", help="Real-Interface (im Sim wird 'lo' erzwungen)")
    parser.add_argument("--window", type=float, default=0.5, help="Log-/Mittelungsfenster [s]")
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()

    domain, iface = resolve_dds(args.interface)
    _log(f"sim_mode={is_sim_mode()}  ->  DDS domain={domain}, interface={iface}")
    ChannelFactoryInitialize(domain, iface)

    st = {"cmd": None, "state": None, "cmd_count": 0, "state_count": 0}
    # Spannweiten-Tracker fuer Soll-q im Fenster
    osc = {"min": {}, "max": {}}

    def reset_osc():
        osc["min"] = {}
        osc["max"] = {}

    def track_osc(idx, q):
        osc["min"][idx] = min(osc["min"].get(idx, q), q)
        osc["max"][idx] = max(osc["max"].get(idx, q), q)

    def cmd_cb(msg: LowCmd_):
        st["cmd"] = msg
        st["cmd_count"] += 1
        for i in LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST:
            track_osc(i, float(msg.motor_cmd[i].q))

    def state_cb(msg: LowState_):
        st["state"] = msg
        st["state_count"] += 1

    sub_cmd = ChannelSubscriber("rt/arm_sdk", LowCmd_)
    sub_cmd.Init(cmd_cb)
    sub_state = ChannelSubscriber("rt/lowstate", LowState_)
    sub_state.Init(state_cb)

    _log("Hoere rt/arm_sdk (Soll) + rt/lowstate (Ist). Jetzt Ziel senden / Marker ziehen ...")

    t0 = time.time()
    last = t0
    last_cmd_c = 0
    while time.time() - t0 < args.seconds:
        time.sleep(args.window)
        now = time.time()
        dt = now - last
        cmd_hz = (st["cmd_count"] - last_cmd_c) / dt if dt > 0 else 0.0
        last, last_cmd_c = now, st["cmd_count"]

        if st["cmd"] is None:
            _log(f"noch KEIN rt/arm_sdk empfangen (arm_controller aktiv? Arme enabled? Ziel gesendet?)"
                 f"  [lowstate rx={st['state_count']}]")
            reset_osc()
            continue

        cmd, state = st["cmd"], st["state"]
        weight = float(cmd.motor_cmd[WEIGHT_IDX].q)

        def arm_report(name, idxs):
            cmd_q = [float(cmd.motor_cmd[i].q) for i in idxs]
            if state is not None:
                meas_q = [float(state.motor_state[i].q) for i in idxs]
                d = [abs(c - m) for c, m in zip(cmd_q, meas_q)]
                dmax = max(d)
            else:
                meas_q = [float("nan")] * len(idxs)
                dmax = float("nan")
            span = [osc["max"].get(i, 0.0) - osc["min"].get(i, 0.0) for i in idxs]
            span_max = max(span) if span else 0.0
            kp = float(cmd.motor_cmd[idxs[0]].kp)
            kd = float(cmd.motor_cmd[idxs[0]].kd)
            _log(f"  {name}: cmd ={_fmt(cmd_q)} kp={kp:.0f} kd={kd:.0f}")
            _log(f"  {name}: meas={_fmt(meas_q)}  d_max={dmax:+.3f}  osc_span_max={span_max:.3f}")

        _log(f"arm_sdk rx_rate={cmd_hz:6.1f} Hz  weight={weight:.2f}")
        arm_report("RIGHT", RIGHT_JOINT_INDICES_LIST)
        arm_report("LEFT ", LEFT_JOINT_INDICES_LIST)
        reset_osc()

    _log("=" * 64)
    _log("Deutung:")
    _log("  osc_span_max gross (z.B. > 0.3) + schnell  -> KOMMANDO oszilliert (IK/Controller)")
    _log("  osc_span_max klein, aber d_max gross        -> Ist folgt nicht (Gains/Physik)")
    _log("  weight ~0 statt 1                           -> arm_sdk-Blending falsch")
    _log("  nur eine Seite mit Bewegung                 -> andere Seite haelt korrekt")


if __name__ == "__main__":
    main()
