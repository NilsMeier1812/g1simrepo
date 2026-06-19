import mujoco
import numpy as np
import pygame
import sys
import struct

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher

from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.utils.thread import RecurrentThread

import config
if config.ROBOT=="g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_GO = 20
NUM_MOTOR_IDL_HG = 35

class UnitreeSdk2Bridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = self.mj_model.nu
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep
        # State-Publish-Rate von der Sim-Rate entkoppeln: bei dt=0.001 (noetig
        # fuer PD-Stabilitaet) wuerde 1 kHz DDS-Verkehr die Realtime-Sim
        # belasten. Auf max. ~500 Hz deckeln (entspricht echtem Roboter).
        self.state_dt = max(self.dt, 0.002)
        self.idl_type = (self.num_motor > NUM_MOTOR_IDL_GO) # 0: unitree_go, 1: unitree_hg

        self.joystick = None

        # Zwei getrennte Befehls-Quellen, damit Loco (Beine/Taille via rt/lowcmd)
        # und Manipulation (Arme via rt/arm_sdk) GLEICHZEITIG laufen koennen.
        # Frueher gab es nur self.low_cmd -> letzte Nachricht gewann -> die jeweils
        # andere Koerperhaelfte ging limp. Der PD-Torque wird pro Sim-Schritt in
        # ApplyLowCmd() gerechnet (nicht im DDS-Callback), damit die Regelrate
        # nicht an der Publish-Rate haengt.
        self.low_cmd_legs = None   # rt/lowcmd  -> Beine+Taille (0..14), ggf. Arme wenn kein arm_sdk
        self.low_cmd_arm = None    # rt/arm_sdk -> Arme (15..28), Weight @ index 29

        # Lockstep-Handshake: cmd_seq zaehlt empfangene rt/lowcmd, state_seq
        # publizierte rt/lowstate. Die Sim-Schleife (unitree_mujoco.py) wartet im
        # Lockstep-Modus auf ein neues cmd_seq, macht dann genau decimation
        # Physikschritte und publiziert EINEN frischen State (state_seq++). So ist
        # die Regelrate an die Sim-Uhr gekoppelt, nicht an die Wall-Clock.
        self.cmd_seq = 0
        self.state_seq = 0
        # Lockstep nur im Loco-Modus (off) sinnvoll. Im Lockstep publiziert die
        # Sim-Schleife den lowstate INLINE nach den decimation-Schritten -> der
        # 500-Hz-RecurrentThread wuerde Zwischenzustaende senden und wird deaktiviert.
        self.lockstep = (
            bool(getattr(config, "SIM_LOCKSTEP", False))
            and str(getattr(config, "HOLD_BASE_MODE", "weld")).lower() == "off"
        )
        self.decimation = int(getattr(config, "SIM_LOCKSTEP_DECIMATION", 20))
        if self.lockstep:
            print(f"[BRIDGE] Lockstep aktiv: {self.decimation} Physikschritte pro "
                  f"rt/lowcmd, lowstate inline danach (deterministische 50-Hz-Regelrate, "
                  f"SIM_REALTIME_FACTOR irrelevant).", flush=True)

        # Loco-Startup-Hold: im off-Modus Beine/Taille in einer Standpose halten,
        # SOLANGE noch kein Loco-Controller auf rt/lowcmd kommandiert hat (legs is
        # None). Ueberbrueckt das Startfenster (colcon build/Launch), in dem der
        # Roboter bei freier Basis sonst umfaellt, bevor loco_sim verbunden ist.
        # Sobald das erste rt/lowcmd ankommt (legs != None), uebernimmt der Regler.
        self.startup_hold_active = (
            str(getattr(config, "HOLD_BASE_MODE", "weld")).lower() == "off"
            and bool(getattr(config, "LOCO_STARTUP_HOLD", True))
        )
        self.startup_hold_pose = list(getattr(config, "LOCO_STARTUP_HOLD_POSE", [0.0] * 15))
        self.startup_hold_kp = float(getattr(config, "LOCO_STARTUP_HOLD_KP", 100.0))
        self.startup_hold_kd = float(getattr(config, "LOCO_STARTUP_HOLD_KD", 2.0))
        if self.startup_hold_active:
            print("[BRIDGE] Loco-Startup-Hold aktiv: halte Beine/Taille in Standpose, "
                  "bis der erste rt/lowcmd-Befehl kommt.", flush=True)

        # Managed-Weld: im off-Modus die Basis (Weld) halten, bis loco_sim das
        # Balancieren startet. loco_sim signalisiert seinen Zustand ueber
        # rt/lowcmd motor_cmd[WEIGHT_IDX].q (Code 0=HOLD, 1=RUN, 2=DAMP). Beim
        # Wechsel nach RUN -> Roboter in saubere Stand-Pose stellen + Weld loesen.
        self.managed_weld = (
            str(getattr(config, "HOLD_BASE_MODE", "weld")).lower() == "off"
            and bool(getattr(config, "LOCO_MANAGED_WELD", True))
        )
        self.weld_id = mujoco.mj_name2id(
            self.mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, "hold_base_weld"
        )
        if self.weld_id < 0:
            self.managed_weld = False
        self._prev_loco_code = 0
        self.stance_pose = list(getattr(config, "LOCO_STARTUP_HOLD_POSE", [0.0] * 15))
        self.reset_z_run = float(getattr(config, "LOCO_RESET_PELVIS_Z", 0.78))
        self.spawn_z = float(self.mj_model.qpos0[2])
        if self.managed_weld:
            print("[BRIDGE] Managed-Weld aktiv: Basis gehalten, bis loco_sim balanciert "
                  "(RUN) -> dann Stand-Pose + Weld loesen.", flush=True)

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name == "imu_quat":
                self.have_imu = True
            if name == "frame_pos":
                self.have_frame_sensor = True

        # Unitree sdk2 message
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(
            interval=self.state_dt, target=self.PublishLowState, name="sim_lowstate"
        )
        # Im Lockstep publiziert die Sim-Schleife den State inline (nach den
        # decimation-Schritten) -> keinen freilaufenden 500-Hz-Publisher starten.
        if not self.lockstep:
            self.lowStateThread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()
        self.HighStateThread = RecurrentThread(
            interval=self.state_dt, target=self.PublishHighState, name="sim_highstate"
        )
        self.HighStateThread.Start()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(
            TOPIC_WIRELESS_CONTROLLER, WirelessController_
        )
        self.wireless_controller_puber.Init()
        self.WirelessControllerThread = RecurrentThread(
            interval=0.01,
            target=self.PublishWirelessController,
            name="sim_wireless_controller",
        )
        self.WirelessControllerThread.Start()

        # rt/lowcmd -> Beine/Taille (Loco-Controller). rt/arm_sdk -> Arme
        # (arm_controller). Getrennte Handler -> getrennte Puffer -> Merge in
        # ApplyLowCmd, statt sich gegenseitig zu ueberschreiben.
        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

        self.arm_sdk_suber = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        self.arm_sdk_suber.Init(self.ArmSdkHandler, 10)

        # joystick
        self.key_map = {
            "R1": 0,
            "L1": 1,
            "start": 2,
            "select": 3,
            "R2": 4,
            "L2": 5,
            "F1": 6,
            "F2": 7,
            "A": 8,
            "B": 9,
            "X": 10,
            "Y": 11,
            "up": 12,
            "right": 13,
            "down": 14,
            "left": 15,
        }

    # Arm-Indizes (15..28) -> aus rt/arm_sdk; alles andere (Beine 0..11, Taille
    # 12..14) -> aus rt/lowcmd. Index 29 = arm_sdk-Weight (Blend Loco<->arm_sdk).
    ARM_LO = 15
    ARM_HI = 28
    WEIGHT_IDX = 29

    def LowCmdHandler(self, msg: LowCmd_):
        # rt/lowcmd (Loco-Controller: Beine/Taille, ggf. ganzer Koerper).
        # Nur speichern; PD-Torque wird in ApplyLowCmd() pro Sim-Schritt gerechnet.
        self.low_cmd_legs = msg
        # Lockstep: signalisiert der Sim-Schleife "neues Kommando da -> jetzt
        # decimation Schritte rechnen". Reihenfolge (erst Daten, dann Zaehler)
        # stellt sicher, dass die Sim beim Aufwachen das fertige msg sieht.
        self.cmd_seq += 1

    def ArmSdkHandler(self, msg: LowCmd_):
        # rt/arm_sdk (arm_controller: Arme + Weight @ index 29).
        self.low_cmd_arm = msg

    @staticmethod
    def _pd(mc, q, dq):
        # Ein Motor-PD-Torque aus einem motor_cmd + aktuellen Sensoren.
        return mc.tau + mc.kp * (mc.q - q) + mc.kd * (mc.dq - dq)

    def _startup_pd(self, i, q, dq):
        # Statischer PD-Hold eines Bein-/Taillen-Motors (0..14) auf die Startpose,
        # solange noch kein Loco-Controller kommandiert.
        target = self.startup_hold_pose[i] if i < len(self.startup_hold_pose) else 0.0
        return self.startup_hold_kp * (target - q) + self.startup_hold_kd * (0.0 - dq)

    def _set_weld(self, active):
        # Weld-Constraint scharf/inaktiv schalten (Laufzeit-Flag eq_active + Startwert).
        val = 1 if active else 0
        if hasattr(self.mj_model, "eq_active0"):
            self.mj_model.eq_active0[self.weld_id] = val
        if hasattr(self.mj_data, "eq_active"):
            self.mj_data.eq_active[self.weld_id] = val

    def _reset_pose(self, legw, pelvis_z):
        # Roboter in eine saubere Pose stellen: Pelvis aufrecht auf pelvis_z (x,y=0),
        # Beine/Taille = legw (15 Werte), ARME = 0, Geschwindigkeit 0. Die Arme MUESSEN
        # mit genullt werden: im gehaltenen HOLD haengen sie sonst limp durch (z.B.
        # Ellbogen ~70 grad), und der Sprung auf 0 beim Policy-Start kippt den Roboter.
        qp = self.mj_data.qpos
        qp[0] = 0.0; qp[1] = 0.0; qp[2] = pelvis_z
        qp[3] = 1.0; qp[4] = 0.0; qp[5] = 0.0; qp[6] = 0.0   # Quaternion aufrecht [w,x,y,z]
        for i in range(self.num_motor):
            qp[7 + i] = legw[i] if i < len(legw) else 0.0
        self.mj_data.qvel[:] = 0.0
        mujoco.mj_forward(self.mj_model, self.mj_data)

    def _handle_managed_weld(self, legs):
        # Zustands-Code von loco_sim aus rt/lowcmd lesen und Basis entsprechend
        # halten/freigeben. 0=HOLD, 1=RUN (aufstehen+frei), 2=DAMP (frei).
        code = int(round(float(legs.motor_cmd[self.WEIGHT_IDX].q))) if legs is not None else 0
        if code == self._prev_loco_code:
            return
        if code == 1:       # Balancing START: in Stand-Pose stellen + Basis freigeben
            self._reset_pose(self.stance_pose, self.reset_z_run)
            self._set_weld(False)
            print("[BRIDGE] Balancing START -> Stand-Pose gesetzt, Weld geloest.", flush=True)
        elif code == 0:     # HOLD/Standby: in Spawn-Pose stellen + Basis halten
            self._reset_pose([0.0] * 15, self.spawn_z)
            self._set_weld(True)
            print("[BRIDGE] HOLD -> Spawn-Pose, Weld an (Basis gehalten).", flush=True)
        elif code == 2:     # DAMP/Emergency: Basis freigeben, kein Reset
            self._set_weld(False)
            print("[BRIDGE] DAMP -> Weld geloest (Basis frei).", flush=True)
        self._prev_loco_code = code

    def ApplyLowCmd(self):
        # Pro Sim-Schritt: Merge der beiden Quellen pro Motor mit AKTUELLEN
        # Sensorwerten -> Regelrate = Sim-Rate, unabhaengig von der Publish-Rate.
        #   Beine/Taille (0..14): aus rt/lowcmd (Loco); solange noch kein Loco-
        #                  Befehl kam -> Startup-Hold (nur off-Modus).
        #   Arme (15..28): aus rt/arm_sdk, per Weight ueber den Loco-Befehl
        #                  geblendet (w=1 -> voll arm_sdk; w=0 -> Loco/aus).
        # Quelle ohne Befehl (kp=kd=tau=0 bzw. None) -> 0 Nm.
        if self.mj_data is None:
            return
        legs = self.low_cmd_legs
        arm = self.low_cmd_arm
        if self.managed_weld:
            self._handle_managed_weld(legs)
        use_startup = legs is None and self.startup_hold_active
        if legs is None and arm is None and not use_startup:
            return

        w = 0.0
        if arm is not None:
            w = float(arm.motor_cmd[self.WEIGHT_IDX].q)
            w = 0.0 if w < 0.0 else (1.0 if w > 1.0 else w)

        n = self.num_motor
        sd = self.mj_data.sensordata
        for i in range(n):
            q = sd[i]
            dq = sd[i + n]
            if self.ARM_LO <= i <= self.ARM_HI and arm is not None:
                tau_a = self._pd(arm.motor_cmd[i], q, dq)
                tau_l = self._pd(legs.motor_cmd[i], q, dq) if legs is not None else 0.0
                self.mj_data.ctrl[i] = w * tau_a + (1.0 - w) * tau_l
            elif legs is not None:
                self.mj_data.ctrl[i] = self._pd(legs.motor_cmd[i], q, dq)
            elif use_startup and i < self.ARM_LO:
                self.mj_data.ctrl[i] = self._startup_pd(i, q, dq)
            else:
                self.mj_data.ctrl[i] = 0.0

    def PublishLowState(self):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.low_state.motor_state[i].q = self.mj_data.sensordata[i]
                self.low_state.motor_state[i].dq = self.mj_data.sensordata[
                    i + self.num_motor
                ]
                self.low_state.motor_state[i].tau_est = self.mj_data.sensordata[
                    i + 2 * self.num_motor
                ]

            if self.have_frame_sensor:

                self.low_state.imu_state.quaternion[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 0
                ]
                self.low_state.imu_state.quaternion[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 1
                ]
                self.low_state.imu_state.quaternion[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 2
                ]
                self.low_state.imu_state.quaternion[3] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 3
                ]

                self.low_state.imu_state.gyroscope[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 4
                ]
                self.low_state.imu_state.gyroscope[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 5
                ]
                self.low_state.imu_state.gyroscope[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 6
                ]

                self.low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 7
                ]
                self.low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 8
                ]
                self.low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[
                    self.dim_motor_sensor + 9
                ]

            if self.joystick != None:
                pygame.event.get()
                # Buttons
                self.low_state.wireless_remote[2] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                0,
                                0,
                                int(self.joystick.get_axis(self.axis_id["LT"]) > 0),
                                int(self.joystick.get_axis(self.axis_id["RT"]) > 0),
                                int(self.joystick.get_button(self.button_id["SELECT"])),
                                int(self.joystick.get_button(self.button_id["START"])),
                                int(self.joystick.get_button(self.button_id["LB"])),
                                int(self.joystick.get_button(self.button_id["RB"])),
                            ]
                        ]
                    ),
                    2,
                )
                self.low_state.wireless_remote[3] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                int(self.joystick.get_hat(0)[0] < 0),  # left
                                int(self.joystick.get_hat(0)[1] < 0),  # down
                                int(self.joystick.get_hat(0)[0] > 0), # right
                                int(self.joystick.get_hat(0)[1] > 0),    # up
                                int(self.joystick.get_button(self.button_id["Y"])),     # Y
                                int(self.joystick.get_button(self.button_id["X"])),     # X
                                int(self.joystick.get_button(self.button_id["B"])),     # B
                                int(self.joystick.get_button(self.button_id["A"])),     # A
                            ]
                        ]
                    ),
                    2,
                )
                # Axes
                sticks = [
                    self.joystick.get_axis(self.axis_id["LX"]),
                    self.joystick.get_axis(self.axis_id["RX"]),
                    -self.joystick.get_axis(self.axis_id["RY"]),
                    -self.joystick.get_axis(self.axis_id["LY"]),
                ]
                packs = list(map(lambda x: struct.pack("f", x), sticks))
                self.low_state.wireless_remote[4:8] = packs[0]
                self.low_state.wireless_remote[8:12] = packs[1]
                self.low_state.wireless_remote[12:16] = packs[2]
                self.low_state.wireless_remote[20:24] = packs[3]

            self.low_state_puber.Write(self.low_state)
            # Lockstep: markiert "frischer State publiziert" fuer den Controller-Takt.
            self.state_seq += 1

    def PublishHighState(self):

        if self.mj_data != None:
            self.high_state.position[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 10
            ]
            self.high_state.position[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 11
            ]
            self.high_state.position[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 12
            ]

            self.high_state.velocity[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 13
            ]
            self.high_state.velocity[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 14
            ]
            self.high_state.velocity[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 15
            ]

        self.high_state_puber.Write(self.high_state)

    def PublishWirelessController(self):
        if self.joystick != None:
            pygame.event.get()
            key_state = [0] * 16
            key_state[self.key_map["R1"]] = self.joystick.get_button(
                self.button_id["RB"]
            )
            key_state[self.key_map["L1"]] = self.joystick.get_button(
                self.button_id["LB"]
            )
            key_state[self.key_map["start"]] = self.joystick.get_button(
                self.button_id["START"]
            )
            key_state[self.key_map["select"]] = self.joystick.get_button(
                self.button_id["SELECT"]
            )
            key_state[self.key_map["R2"]] = (
                self.joystick.get_axis(self.axis_id["RT"]) > 0
            )
            key_state[self.key_map["L2"]] = (
                self.joystick.get_axis(self.axis_id["LT"]) > 0
            )
            key_state[self.key_map["F1"]] = 0
            key_state[self.key_map["F2"]] = 0
            key_state[self.key_map["A"]] = self.joystick.get_button(self.button_id["A"])
            key_state[self.key_map["B"]] = self.joystick.get_button(self.button_id["B"])
            key_state[self.key_map["X"]] = self.joystick.get_button(self.button_id["X"])
            key_state[self.key_map["Y"]] = self.joystick.get_button(self.button_id["Y"])
            key_state[self.key_map["up"]] = self.joystick.get_hat(0)[1] > 0
            key_state[self.key_map["right"]] = self.joystick.get_hat(0)[0] > 0
            key_state[self.key_map["down"]] = self.joystick.get_hat(0)[1] < 0
            key_state[self.key_map["left"]] = self.joystick.get_hat(0)[0] < 0

            key_value = 0
            for i in range(16):
                key_value += key_state[i] << i

            self.wireless_controller.keys = key_value
            self.wireless_controller.lx = self.joystick.get_axis(self.axis_id["LX"])
            self.wireless_controller.ly = -self.joystick.get_axis(self.axis_id["LY"])
            self.wireless_controller.rx = self.joystick.get_axis(self.axis_id["RX"])
            self.wireless_controller.ry = -self.joystick.get_axis(self.axis_id["RY"])

            self.wireless_controller_puber.Write(self.wireless_controller)

    def SetupJoystick(self, device_id=0, js_type="xbox"):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(device_id)
            self.joystick.init()
        else:
            print("[WARN] No gamepad detected — joystick disabled, simulation continues.", flush=True)
            return

        if js_type == "xbox":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 2,
                "Y": 3,
                "B": 1,
                "A": 0,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
            }

        elif js_type == "switch":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 2,  # Right stick axis x
                "RY": 3,  # Right stick axis y
                "LT": 5,  # Left trigger
                "RT": 4,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 3,
                "Y": 4,
                "B": 1,
                "A": 0,
                "LB": 6,
                "RB": 7,
                "SELECT": 10,
                "START": 11,
            }
        else:
            print("Unsupported gamepad. ")

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.mj_model.sensor_dim[i],
                )
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
