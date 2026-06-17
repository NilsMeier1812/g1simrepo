"""
hold_base.py — kapselt das "Oberkoerper ruhig halten" fuer Arm-Tests ohne
Loco-/Balance-Controller. Wird von unitree_mujoco.py UND vom Physik-Testtool
(sim_hold_test.py) genutzt, damit beide exakt dieselbe Basis-Behandlung haben.

Modi (config.HOLD_BASE_MODE):
  "weld"     : torso_link per Weld-Constraint (in scene.xml definiert) starr an
               die Welt + Beine/Taille als steife Gelenkfedern. Impulserhaltend,
               KEIN Teleport -> sauberste, jitterfreie Basis fuer die Arme.
  "teleport" : Becken/Beine/Taille werden jeden Sim-Schritt hart auf qpos0
               gesetzt (Weld deaktiviert). Legacy; kann Zittern in die Arme pumpen.
  "off"      : Basis frei (Weld deaktiviert).
"""
import mujoco


WELD_NAME = "hold_base_weld"

# Bein- + Taillen-Gelenke (alles ausser Armen + freier Basis).
LEG_WAIST_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
]

# qpos 0:22 = freie Basis(7) + Beine(12) + Taille(3); 22:36 = Arme(14).
LOWER_BODY_QPOS = slice(0, 22)
LOWER_BODY_QVEL = slice(0, 21)


def resolve_mode(config):
    """Effektiven HOLD_BASE-Modus aus der config bestimmen."""
    mode = getattr(config, "HOLD_BASE_MODE", None)
    if mode is None:
        mode = "weld" if getattr(config, "HOLD_BASE", False) else "off"
    mode = str(mode).lower()
    # HOLD_BASE=False erzwingt "off" (Rueckwaerts-Kompatibilitaet).
    if not getattr(config, "HOLD_BASE", True):
        mode = "off"
    return mode


class HoldBase:
    def __init__(self, mj_model, config, mj_data=None):
        # mj_data wird gebraucht, um die LAUFZEIT-Flag eq_active zu setzen. Sonst
        # bleibt der Weld im off/teleport-Modus aktiv (eq_active0 wirkt nur bei
        # mj_resetData) und der Roboter haengt starr in der Luft.
        self.mj_data = mj_data
        self.mode = resolve_mode(config)
        self.stiffness = float(getattr(config, "HOLD_BASE_STIFFNESS", 2000.0))
        self.damping = float(getattr(config, "HOLD_BASE_DAMPING", 80.0))
        self._qpos0_lower = mj_model.qpos0[LOWER_BODY_QPOS].copy()

        if self.mode == "weld":
            self._set_weld_active(mj_model, True)
            self._stiffen_lower_body(mj_model)
            print(f"[HOLD_BASE] Modus=weld: torso_link verschweisst + Beine/Taille "
                  f"versteift (k={self.stiffness:.0f}, d={self.damping:.0f}), kein Teleport.",
                  flush=True)
        elif self.mode == "teleport":
            self._set_weld_active(mj_model, False)
            print("[HOLD_BASE] Modus=teleport: Unterkoerper wird jeden Step auf qpos0 "
                  "gesetzt (Weld aus).", flush=True)
        else:
            managed = bool(getattr(config, "LOCO_MANAGED_WELD", True))
            if managed:
                # Basis vorerst gehalten lassen; die Bridge loest den Weld erst,
                # wenn loco_sim das Balancieren startet (RUN). Beine bleiben frei
                # (nicht versteift) -> der Loco-Controller regelt sie.
                self._set_weld_active(mj_model, True)
                print("[HOLD_BASE] Modus=off (managed): Basis vorerst gehalten (Weld an); "
                      "Freigabe, sobald loco_sim balanciert.", flush=True)
            else:
                self._set_weld_active(mj_model, False)
                print("[HOLD_BASE] Modus=off: Basis frei (Weld aus).", flush=True)

    def _set_weld_active(self, mj_model, active):
        try:
            wid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, WELD_NAME)
            if wid < 0:
                print(f"[HOLD_BASE] WARN: Weld '{WELD_NAME}' nicht im Modell gefunden.",
                      flush=True)
                return
            val = 1 if active else 0
            # eq_active0 (MuJoCo >= 2.3.2) bzw. eq_active (aeltere Versionen).
            if hasattr(mj_model, "eq_active0"):
                mj_model.eq_active0[wid] = val
            elif hasattr(mj_model, "eq_active"):
                mj_model.eq_active[wid] = val
            else:
                print("[HOLD_BASE] WARN: keine eq_active(0)-API gefunden.", flush=True)
            # KRITISCH: eq_active0 ist nur der Startwert, den mj_resetData nach
            # mj_data.eq_active kopiert. Der Solver liest aber pro Step die
            # LAUFZEIT-Flag mj_data.eq_active. Diese wurde bei der MjData-Erzeugung
            # bereits gelatcht -> ohne folgendes Setzen bleibt der Weld trotz
            # eq_active0=0 aktiv (Roboter haengt in der Luft). Darum hier direkt
            # die Laufzeit-Flag mitschalten.
            if self.mj_data is not None and hasattr(self.mj_data, "eq_active"):
                self.mj_data.eq_active[wid] = val
        except Exception as e:
            print(f"[HOLD_BASE] WARN: Weld-Schalten fehlgeschlagen: {e}", flush=True)

    def _stiffen_lower_body(self, mj_model):
        for name in LEG_WAIST_JOINTS:
            try:
                jid = mj_model.joint(name).id
                mj_model.jnt_stiffness[jid] = self.stiffness
                dofadr = mj_model.jnt_dofadr[jid]
                mj_model.dof_damping[dofadr] = self.damping
            except Exception as e:
                print(f"[HOLD_BASE] WARN: konnte {name} nicht versteifen: {e}", flush=True)

    def after_step(self, mj_data):
        """Nach jedem mj_step aufrufen. Nur im teleport-Modus aktiv."""
        if self.mode == "teleport":
            mj_data.qpos[LOWER_BODY_QPOS] = self._qpos0_lower
            mj_data.qvel[LOWER_BODY_QVEL] = 0
