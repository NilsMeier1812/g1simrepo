# Herkunft & Lizenz der Whole-Body-Policy

`policy.onnx` ist die **G1 Velocity-Policy** aus dem offiziellen Unitree-Repo
[`unitreerobotics/unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab),
Pfad `deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx`.

* **Lizenz:** Apache License 2.0 (siehe `LICENSE` in diesem Ordner). Die Apache-2.0
  erlaubt Weitergabe (auch der unveraenderten `.onnx`) mit Beibehaltung von Lizenz +
  Hinweis. Die Datei ist **unveraendert** uebernommen.
* **deploy.yaml:** ebenfalls 1:1 aus dem Quell-Repo (autoritative Deploy-Config:
  Gelenk-Reihenfolge, kp/kd, Default-Pose, Action-Scales, Obs-Layout, Kommandobereiche).

## Warum diese Policy

Velocity-konditionierte **Ganzkoerper**-Policy (98 Obs -> 29 Aktionen, alle Gelenke).
Mit `rel_standing_envs=0.05` trainiert -> der `gait_phase`-Obs-Term wird bei
`||cmd|| < 0.1` auf 0 gesetzt = **Stand-Signal**. Dadurch:

* **cmd = 0  -> steht still** (headless validiert: ~0.01 m Drift / 10 s).
* **cmd != 0 -> laeuft** (vor/zurueck/seit/drehen).
* **Walk -> Stand** = einfach `cmd -> 0` (7/8 Richtungen sauber, kein Stepping-Stop,
  kein modellbasierter PD, kein `reserve[]`-Schmuggel noetig).

Braucht nur **IMU + Gelenk-Encoder** (keine Fusssensoren). Ersetzt die fruehere
velocity-blinde `motion.pt` + den modellbasierten PD-Balancer.

## Obs-Layout (98), 1:1 aus deploy.yaml (alle Scales 1.0)

| Bereich | Groesse | Inhalt |
|---|---|---|
| 0:3   | 3  | base_ang_vel (Gyro, Body-Frame) |
| 3:6   | 3  | projected_gravity (aufrecht = [0,0,-1]) |
| 6:9   | 3  | velocity_command (vx, vy, vyaw) |
| 9:11  | 2  | gait_phase (sin, cos; **0,0 wenn ‖cmd‖<0.1** -> stehen) |
| 11:40 | 29 | joint_pos_rel = q - default_joint_pos |
| 40:69 | 29 | joint_vel (= dq) |
| 69:98 | 29 | last_action |

Aktion -> Ziel: `q_target[k] = default_joint_pos[k] + action[k] * action_scale[k]`,
dann PD je Gelenk mit `stiffness/damping` aus `deploy.yaml`. Regelrate 50 Hz
(step_dt 0.02), gait_period 0.6 s.
