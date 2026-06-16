import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config


locker = threading.Lock()

# === HOLD_BASE STATE START ===
_hold_base_initial_pose = None
# === HOLD_BASE STATE END ===

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

# Der HOLD_BASE-Weld (torso_link, in scene.xml) ist per Default aktiv. Wenn
# HOLD_BASE aus ist (echter Loco-Controller uebernimmt), deaktivieren wir ihn,
# damit die Basis frei ist.
if not getattr(config, 'HOLD_BASE', False):
    try:
        _wid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, 'hold_base_weld')
        if _wid >= 0:
            mj_model.eq_active0[_wid] = 0
    except Exception as _e:
        print(f'[HOLD_BASE] Konnte Weld nicht deaktivieren: {_e}', flush=True)


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
        mujoco.mj_step(mj_model, mj_data)

        # === HOLD_BASE HOOK START ===
        # Untere Koerperhaelfte (Basis + Beine + Taille, qpos 0:22) auf die
        # Referenz-Standpose qpos0 einfrieren. qpos0 ist zugleich der Anker des
        # torso_link-Weld (scene.xml) -> beide sind konsistent, nichts arbeitet
        # gegeneinander. Die Arme (qpos 22:36) bleiben frei und werden vom
        # arm_controller geregelt; ihre starre Basis liefert der Weld.
        if getattr(config, 'HOLD_BASE', False):
            global _hold_base_initial_pose
            if _hold_base_initial_pose is None:
                _hold_base_initial_pose = mj_model.qpos0[0:22].copy()
                print('[HOLD_BASE] Unterkoerper auf Standpose (qpos0) eingefroren; '
                      'torso_link per Weld starr.', flush=True)
            mj_data.qpos[0:22] = _hold_base_initial_pose
            mj_data.qvel[0:21] = 0
        # === HOLD_BASE HOOK END ===

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


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
