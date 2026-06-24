import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config
from hold_base import HoldBase
from push_listener import PushListener


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

# HOLD_BASE: Oberkoerper fuer Arm-Tests ruhig halten (bis ein Loco-Controller
# existiert). Modus/Tuning kommen aus config.py. Richtet Weld/Steifigkeit einmal
# ein; im teleport-Modus liefert step() einen per-Step-Hook.
hold_base = HoldBase(mj_model, config, mj_data)

# PUSH: Stoer-Impuls fuer den Balancer-Test. Hoert auf UDP (vom Streamdeck-Button
# ueber loco_sim) und bringt eine kurze Kraft in zufaelliger Richtung auf.
push = PushListener(mj_model, config)

if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    if getattr(unitree, "lockstep", False):
        SimulationLockstep(unitree)
        return

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        # PD-Torque JEDEN Schritt mit aktuellen Sensoren neu rechnen, damit die
        # Regelrate = Sim-Rate ist und nicht an der (evtl. langsamen) Publish-
        # Rate von rt/lowcmd / rt/arm_sdk haengt (sonst Open-Loop-Torque zwischen
        # Nachrichten -> Aufschwingen der Arme).
        unitree.ApplyLowCmd()
        push.apply(mj_data)          # Stoer-Impuls VOR dem Step setzen (wirkt im Step)
        mujoco.mj_step(mj_model, mj_data)

        # Nur im teleport-Modus: Unterkoerper jeden Schritt zuruecksetzen.
        hold_base.after_step(mj_data)

        # Sensoren auf den AKTUELLEN Zustand bringen: mj_step fuellt sensordata vor
        # der Integration -> sonst publiziert der lowStateThread einen Schritt alte
        # Geschwindigkeiten (dq/gyro), an denen eine RL-Policy kippt. Siehe Lockstep.
        mujoco.mj_forward(mj_model, mj_data)

        locker.release()

        # Realtime-Faktor: Sim absichtlich langsamer als Echtzeit laufen lassen
        # (config.SIM_REALTIME_FACTOR < 1), damit eine 50-Hz-Policy auf langsamen
        # PCs pro Schritt wieder die volle Physik bekommt. 1.0 = Echtzeit.
        factor = getattr(config, "SIM_REALTIME_FACTOR", 1.0)
        if factor <= 0:
            factor = 1.0
        time_until_next_step = mj_model.opt.timestep / factor - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def SimulationLockstep(unitree):
    """Deterministische Sim: GENAU decimation Physikschritte pro empfangenem
    rt/lowcmd, danach EIN frischer rt/lowstate. Die Regelrate ist damit an die
    Sim-Uhr gekoppelt (nicht Wall-Clock) -> eine 50-Hz-Policy sieht auf jedem PC
    exakt ihre trainierten 20 ms Physik/Schritt.

    ECHTZEIT-PACING: pro Regelzyklus werden decimation*timestep Sekunden SIM-Zeit
    simuliert; der Zyklus wird auf die entsprechende WANDZEIT (geteilt durch
    SIM_REALTIME_FACTOR) gestreckt. Sonst laeuft die Sim so schnell wie die CPU kann
    (auf schnellen PCs 2-3x Echtzeit -> wirkt wie ein vorgespultes Video). Das Pacing
    DECKELT nur die Geschwindigkeit; auf langsamen PCs darf der Zyklus laenger dauern
    (Physik/Schritt bleibt identisch -> Determinismus unangetastet). factor<=0 -> kein
    Pacing (so schnell wie moeglich).

    Watchdog: kommt kein neues Kommando (Controller noch nicht verbunden oder
    abgestuerzt), wird nach SIM_LOCKSTEP_WATCHDOG_S trotzdem geschritten, damit
    Sim/Viewer nie hart einfrieren (Startfenster + Robustheit)."""
    global mj_data, mj_model
    decimation = int(getattr(config, "SIM_LOCKSTEP_DECIMATION", 20))
    watchdog = float(getattr(config, "SIM_LOCKSTEP_WATCHDOG_S", 0.2))
    factor = float(getattr(config, "SIM_REALTIME_FACTOR", 1.0))
    # Soll-Wandzeit pro Regelzyklus (decimation Schritte = SIM_CONTROL_DT Sim-Zeit).
    cycle_wall = (decimation * mj_model.opt.timestep / factor) if factor > 0 else 0.0
    last_cmd_seq = -1
    while viewer.is_running():
        cycle_start = time.perf_counter()
        # Auf ein NEUES Kommando warten (oder Watchdog), ohne die CPU zu blockieren.
        t_wait = time.perf_counter()
        while (unitree.cmd_seq == last_cmd_seq
               and (time.perf_counter() - t_wait) < watchdog
               and viewer.is_running()):
            time.sleep(0.0002)
        last_cmd_seq = unitree.cmd_seq

        locker.acquire()
        if config.ENABLE_ELASTIC_BAND and elastic_band.enable:
            mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                mj_data.qpos[:3], mj_data.qvel[:3]
            )
        # Ein Regelschritt = decimation Physikschritte; PD pro Schritt mit frischen
        # Sensoren (das gehaltene Kommando bleibt konstant -> wie auf der Hardware,
        # wo der Low-Level-PD zwischen 50-Hz-Sollwerten mit 1 kHz weiterregelt).
        for _ in range(decimation):
            unitree.ApplyLowCmd()
            push.apply(mj_data)      # Stoer-Impuls VOR dem Step setzen (wirkt im Step)
            mujoco.mj_step(mj_model, mj_data)
            hold_base.after_step(mj_data)
        # KRITISCH: mj_step fuellt sensordata am ANFANG des Schritts (vor der
        # Integration) -> nach der Schleife sind die Sensoren (v.a. Gelenk-/IMU-
        # GESCHWINDIGKEITEN) einen Schritt ALT. Bei Fussaufprall weicht dq dadurch
        # um >1 rad/s vom wahren qvel ab; eine RL-Policy, die genau diese dq/gyro in
        # ihrer Observation hat, kippt davon. mj_forward rechnet die Sensoren ohne
        # weitere Integration auf den AKTUELLEN Zustand neu -> sensordata == qpos/qvel
        # (headless verifiziert: Diff 1.44 -> 0.0). Erst danach publizieren.
        mujoco.mj_forward(mj_model, mj_data)
        locker.release()

        # Frischen State NACH den Schritten publizieren -> der Controller reagiert
        # immer auf den Post-Step-Zustand (strikte 1:1-Alternation).
        unitree.PublishLowState()

        # Echtzeit-Pacing: Rest des Zyklus bis zur Soll-Wandzeit abwarten, damit die
        # Sim NIE schneller als Echtzeit laeuft (deckelt, verlangsamt nie die Physik).
        if cycle_wall > 0.0:
            remaining = cycle_wall - (time.perf_counter() - cycle_start)
            if remaining > 0:
                time.sleep(remaining)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
