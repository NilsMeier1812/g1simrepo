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
    import threading as _t
    print(f'[DIAG] tid_sim={_t.get_ident()}', flush=True)
    print(f'[DIAG] SAME_OBJECT={mj_data is unitree.mj_data}', flush=True)
    print(f'[DIAG] outer_id={id(mj_data)} bridge_id={id(unitree.mj_data)}', flush=True)
    print(f'[DIAG] outer_ctrl_ptr={mj_data.ctrl.ctypes.data}', flush=True)
    print(f'[DIAG] bridge_ctrl_ptr={unitree.mj_data.ctrl.ctypes.data}', flush=True)
    print(f'[DIAG] num_motor={mj_model.nu} ctrl_len={len(mj_data.ctrl)}', flush=True)
    print(f'[DIAG] actuator_gaintype={list(mj_model.actuator_gaintype[:5])}...', flush=True)
    print(f'[DIAG] actuator_biastype={list(mj_model.actuator_biastype[:5])}...', flush=True)

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
        _ctrl22_before = float(mj_data.ctrl[22])
        # Print 1x pro Sekunde damit wir Pointer + tid sehen
        import threading as _tt
        if not hasattr(SimulationThread, '_last_diag') or (time.time() - SimulationThread._last_diag) > 1.0:
            SimulationThread._last_diag = time.time()
            print(f'[SIM] tid={_tt.get_ident()} mj_data_id={id(mj_data)} ctrl_ptr={mj_data.ctrl.ctypes.data} ctrl22={_ctrl22_before:.3f}', flush=True)
        try:
            from unitree_sdk2py_bridge import _g_ctrl, _g_ctrl_ready
            if _g_ctrl_ready and _g_ctrl is not None:
                mj_data.ctrl[:len(_g_ctrl)] = _g_ctrl
        except ImportError:
            pass
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
                _hold_base_initial_pose = mj_data.qpos[0:7].copy()
                mj_data.qvel[:] = 0
                print(f'[HOLD_BASE] Standing pose geladen:', flush=True)
                print(f'  pelvis z={mj_data.qpos[2]:.4f}', flush=True)
                print(f'  left foot z={mj_data.xpos[mj_model.body("left_ankle_roll_link").id][2]:.4f}', flush=True)
                print(f'  right foot z={mj_data.xpos[mj_model.body("right_ankle_roll_link").id][2]:.4f}', flush=True)
            # Nur Pelvis pose + velocity fixieren — Arme/Beine frei!
            mj_data.qpos[0:7] = _hold_base_initial_pose
            mj_data.qvel[0:6] = 0
        # === HOLD_BASE HOOK END ===

        _ctrl22_after = float(mj_data.ctrl[22])
        if abs(_ctrl22_before) > 0.001:
            print(f'[SIM] before={_ctrl22_before:.3f} after={_ctrl22_after:.3f} sensor={mj_data.sensordata[22]:.4f}', flush=True)

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
