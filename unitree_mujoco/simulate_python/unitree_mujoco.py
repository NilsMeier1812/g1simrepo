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
        # Steuerbefehle werden von UnitreeSdk2Bridge.LowCmdHandler direkt in
        # mj_data.ctrl geschrieben (rt/lowcmd + rt/arm_sdk).
        mujoco.mj_step(mj_model, mj_data)

        # === HOLD_BASE HOOK START ===
        if getattr(config, 'HOLD_BASE', False):
            global _hold_base_initial_pose
            if _hold_base_initial_pose is None:
                # G1 Standing Pose (qpos indices from joint query)
                # Base: pos=[0, 0, z_stand], quat=[1,0,0,0]
                mj_data.qpos[0:3] = [0.0, 0.0, 0.75]
                mj_data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
                # Legs: slight knee bend for natural stance
                #   hip_pitch(7,13) knee(10,16) ankle_pitch(11,17)
                mj_data.qpos[7]  = -0.15   # left_hip_pitch
                mj_data.qpos[10] =  0.30   # left_knee
                mj_data.qpos[11] = -0.15   # left_ankle_pitch
                mj_data.qpos[13] = -0.15   # right_hip_pitch
                mj_data.qpos[16] =  0.30   # right_knee
                mj_data.qpos[17] = -0.15   # right_ankle_pitch
                # Arms: natural at sides
                mj_data.qpos[22] =  0.3    # left_shoulder_pitch (leicht vor)
                mj_data.qpos[23] =  0.15   # left_shoulder_roll (leicht seitlich)
                mj_data.qpos[25] =  0.5    # left_elbow (leicht gebeugt)
                mj_data.qpos[29] =  0.3    # right_shoulder_pitch
                mj_data.qpos[30] = -0.15   # right_shoulder_roll (seitlich, gespiegelt)
                mj_data.qpos[32] =  0.5    # right_elbow
                # Vorwärts-Kinematik berechnen damit Sensoren stimmen
                import mujoco as _mj
                _mj.mj_forward(mj_model, mj_data)
                # Fuss-Höhe prüfen und Pelvis anpassen
                left_foot_z = mj_data.xpos[mj_model.body('left_ankle_roll_link').id][2]
                right_foot_z = mj_data.xpos[mj_model.body('right_ankle_roll_link').id][2]
                min_foot_z = min(left_foot_z, right_foot_z)
                # Pelvis anheben sodass Füsse knapp über Boden schweben (~2cm)
                mj_data.qpos[2] += (0.02 - min_foot_z)
                _mj.mj_forward(mj_model, mj_data)
                # qpos 0:7 = freie Basis, 7:22 = Beine(12) + Taille(3),
                # 22:36 = Arme(14). Wir frieren Basis + Beine + Taille ein,
                # damit der Roboter ohne Loco-Controller ruhig steht.
                _hold_base_initial_pose = mj_data.qpos[0:22].copy()
                mj_data.qvel[:] = 0
                print(f'[HOLD_BASE] Standing pose geladen (pelvis z={mj_data.qpos[2]:.4f}).', flush=True)
            # Basis + Beine + Taille fixieren — nur die Arme (qpos 22:36) bleiben frei.
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
