#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_dds_check — Standalone-Diagnose: empfängt g1pilot LowState von MuJoCo?

Prüft die unterste Schicht (DDS-Verbindung g1pilot <-> MuJoCo-Bridge) komplett
unabhängig von ROS 2, IK oder RViz. Initialisiert die Domain/Interface über
denselben Sim/Real-Switch wie die Nodes (G1_SIM_MODE) und meldet sekündlich, ob
und wie schnell rt/lowstate ankommt, plus mode_machine und die Arm-Gelenkwinkel.

Aufruf (MuJoCo muss laufen):
    G1_SIM_MODE=true python3 -m g1pilot.tools.sim_dds_check
    # oder, nach colcon build + source:
    G1_SIM_MODE=true ros2 run g1pilot sim_dds_check

ECHTER ROBOTER (Alias `dds_check`, erster Verbindungstest — s. g1pilot/docs/70_echtroboter_anleitung.md):
    G1_SIM_MODE=false ros2 run g1pilot dds_check --interface <NIC>
Erwartung: rt/lowstate mit ~500 Hz und plausiblen Gelenkwinkeln.
"""
import argparse
import time
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from g1pilot.utils.common import resolve_dds, is_sim_mode
from g1pilot.utils.joints_names import RIGHT_JOINT_INDICES_LIST, LEFT_JOINT_INDICES_LIST


def _log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] [sim_dds_check] {msg}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="eth0",
                        help="Real-Interface (im Sim wird 'lo' erzwungen)")
    parser.add_argument("--seconds", type=float, default=15.0)
    args = parser.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    domain, iface = resolve_dds(args.interface)
    _log(f"sim_mode={is_sim_mode()}  ->  DDS domain={domain}, interface={iface}")
    ChannelFactoryInitialize(domain, iface)

    state = {"count": 0, "last": None}

    def cb(msg: LowState_):
        state["count"] += 1
        state["last"] = msg

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(cb)
    _log("Warte auf rt/lowstate ... (MuJoCo gestartet? richtige Domain?)")

    t0 = time.time()
    last_tick = t0
    last_count = 0
    got_any = False
    while time.time() - t0 < args.seconds:
        time.sleep(1.0)
        now = time.time()
        c = state["count"]
        hz = (c - last_count) / (now - last_tick)
        last_tick, last_count = now, c
        if c == 0:
            _log("... noch KEINE LowState-Nachricht empfangen.")
            continue
        got_any = True
        m = state["last"]
        mode_machine = getattr(m, "mode_machine", "n/a")
        rq = [round(float(m.motor_state[i].q), 3) for i in RIGHT_JOINT_INDICES_LIST]
        lq = [round(float(m.motor_state[i].q), 3) for i in LEFT_JOINT_INDICES_LIST]
        _log(f"rx={c:5d}  rate={hz:6.1f} Hz  mode_machine={mode_machine}")
        _log(f"   right_arm q = {rq}")
        _log(f"   left_arm  q = {lq}")

    _log("=" * 60)
    if got_any:
        _log(f"ERGEBNIS: OK — LowState empfangen ({state['count']} Nachrichten).")
    else:
        _log("ERGEBNIS: FEHLER — keine LowState-Daten. Prüfe: MuJoCo läuft? "
             "config.py DOMAIN_ID=1/INTERFACE=lo? G1_SIM_MODE=true gesetzt?")


if __name__ == "__main__":
    main()
