# G1 Box-Carry Policy — Trainings-Overlay

Trainiert eine eigene G1-Velocity-Policy, die **läuft, bei v=0 steht — ohne die
Arme zum Balancieren zu brauchen — und die Arme in beliebigen Posen *vor dem
Körper* hält** (leere Kiste tragen). Das ist ein dünner Overlay auf das offizielle
[`unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab) (mjlab +
mujoco-warp, PPO/RSL-RL).

> **Warum die Stock-Policy das nicht kann:** Ihre `pose`-Reward zieht die Arme zur
> Default-Pose, und sie kennt kein Arm-Posen-Kommando und keine Arm-Randomization.
> Dieser Overlay ändert genau diese drei Dinge.

---

## Was der Overlay hinzufügt

| Datei (neu) | Inhalt |
|---|---|
| `mdp/arm_pose_command.py` | **Arm-Pose-Kommando**: sampelt pro Episode eine Ziel-Armpose aus einem **Front-Envelope** (Delta um die Default-Pose; Schulter nur nach vorn, Ellbogen nur gebeugt). Curriculum-skaliert. |
| `mdp/boxcarry_rewards.py` | **`arm_pose_tracking`**-Reward: Arme folgen dem Kommando. |
| `mdp/boxcarry_curriculum.py` | **`arm_pose_levels`**: weitet den Arm-Envelope über das Training auf (0.3 → 1.0). |
| `config/g1_boxcarry/` | Task **`Unitree-G1-BoxCarry-Flat`**: baut auf `Unitree-G1-Flat`, hängt Kommando/Obs/Reward/Event/Curriculum ein und **befreit** `pose`+`stand_still` von den Armen. |

Geänderte Reward-Logik (alles andere = Stock-Walker, unverändert):
- `pose` und `stand_still` wirken nur noch auf **Beine + Taille**.
- **`arm_pose_tracking`** regelt die Arme auf die kommandierte Front-Pose.
- **`carry_payload`**-Event: kleiner Vorwärts-CoM-Offset am Torso (Proxy für die leere Kiste).

---

## 1. Voraussetzungen / Was du brauchst

- **NVIDIA-GPU** (deine RTX 4090 passt; mujoco-warp braucht CUDA). 24 GB reichen für 4096 Envs (Flat). Bei OOM: `--env.scene.num-envs=2048`.
- **Linux + aktuelle NVIDIA-Treiber + CUDA** (Treiber ≥ für CUDA 12).
- Python 3.10/3.11, ein frisches venv/conda-Env.
- Das offizielle Repo (wird unten geklont). **Die Policy selbst musst du nichts „herunterladen"** — sie wird beim Training erzeugt; `policy.onnx` wird automatisch exportiert.

## 2. Setup (einmalig)

```bash
# a) Offizielles Trainings-Repo klonen
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git
cd unitree_rl_mjlab

# b) Env anlegen + installieren (zieht mjlab + mujoco-warp + rsl_rl)
python -m venv .venv && source .venv/bin/activate
pip install --upgrade pip
pip install -e .

# c) Smoke-Test: Stock-Task muss auflisten
python scripts/list_envs.py | grep -i g1
```

## 3. Overlay anwenden

Dieser `training/`-Ordner liegt in deinem g1simrepo (Branch `training`). Aus dem
g1simrepo heraus:

```bash
cd /pfad/zu/g1simrepo/training
./apply_overlay.sh /pfad/zu/unitree_rl_mjlab
```

Das kopiert die 4 neuen Dateien und ergänzt `mdp/__init__.py` um die Importe.
**Verifizieren:**

```bash
cd /pfad/zu/unitree_rl_mjlab
python scripts/list_envs.py | grep -i boxcarry      # -> Unitree-G1-BoxCarry-Flat
```

## 4. Training starten

**Ein einziger Lauf trainiert das komplette Zielszenario** (Laufen + Stehen bei v=0
ohne Arm-Abhängigkeit + Front-Armposen + leere Kiste). Es gibt **keine manuellen
Phasen**: das `arm_pose_levels`-Curriculum weitet den Arm-Bereich **automatisch
innerhalb dieses Laufs** auf (≈Iter 0→4000) und trainiert danach das volle Ziel.

```bash
python scripts/train.py Unitree-G1-BoxCarry-Flat --env.scene.num-envs=4096

# (optional) Multi-GPU
python scripts/train.py Unitree-G1-BoxCarry-Flat --gpu-ids 0 1 --env.scene.num-envs=4096
```

- Dauer: grob **3–8 h** für 15 000 Iterationen auf einer 4090 (Schätzung).
- Logs/Checkpoints: `logs/rsl_rl/g1_boxcarry/<datum_zeit>/model_<iter>.pt`
- **`policy.onnx` wird während des Trainings automatisch mitexportiert.**

> **Nur** falls du SPÄTER eine **schwere/volle** Kiste willst (nicht für die leere
> nötig): im `env_cfgs.py` die `carry_payload`-Ranges hochziehen und mit einem
> guten Checkpoint dieses Laufs warm-starten. Für die leere Kiste ist hier nichts
> zu tun — sie ist bereits Teil dieses einen Laufs.

## 5. Training beobachten (worauf achten)

TensorBoard:
```bash
tensorboard --logdir logs/rsl_rl/g1_boxcarry
```

Die **entscheidenden Kurven**:
| Metrik | Erwartung „läuft gut" |
|---|---|
| `Episode/rew_track_linear_velocity`, `..._angular_velocity` | steigt, sättigt hoch |
| `Episode/rew_arm_pose_tracking` | steigt; Arme folgen der Pose |
| `Metrics/arm_pose/arm_pose_error` | **sinkt** über Training |
| `Episode/termination_fell_over` (oder Episodenlänge) | Stürze **sinken**, Episodenlänge → max |
| `Curriculum/arm_pose_levels/arm_delta_scale` | steigt stufig 0.3→0.6→1.0 |

**Gesund:** Velocity-Tracking steigt zuerst, Arm-Tracking zieht nach, Stürze fallen,
während der Arm-Envelope sich weitet. **Warnsignal:** Stürze schnellen hoch, sobald
`arm_delta_scale` steigt → Curriculum langsamer machen (siehe Tuning).

## 6. Ansehen / prüfen

```bash
python scripts/play.py Unitree-G1-BoxCarry-Flat --num-envs=16
# (genaue Flags: python scripts/play.py --help; Checkpoint wird i.d.R. autom. gewählt)
```
Im Play-Mode ist der volle Front-Envelope aktiv — die Arme springen in
verschiedene Front-Posen, der Roboter soll stehen/laufen ohne zu fallen.

## 7. Deployen in deine Sim (g1pilot)

Die neue Policy hat eine **größere Observation** (Arm-Kommando hängt hinten an):

```
NEU (112):  [ base_ang_vel(3) | proj_gravity(3) | twist_cmd(3) | phase(2) |
              joint_pos_rel(29) | joint_vel(29) | last_action(29) | ARM_POSE_CMD(14) ]
```
`ARM_POSE_CMD` = die 14 **absoluten** Arm-Gelenk-Sollwerte (rad), in Modell-Joint-
Reihenfolge der Arm-Regexe (shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw,
links vor rechts — wie `find_joints` sie liefert; die exportierten ONNX-Metadaten
`joint_names` zeigen die Reihenfolge).

In `loco_sim.py` musst du dann:
1. die Obs um diese 14 Werte **am Ende** erweitern,
2. als Arm-Kommando die **gewünschte Tragepose** einspeisen (dieselbe, die der
   `arm_controller` hält),
3. die Policy-Arm-Ausgänge entweder anwenden **oder** weiter vom `arm_controller`
   überschreiben lassen — die Beine balancieren in beiden Fällen, weil sie auf
   genau diese Front-Posen trainiert wurden.

> Der Deploy-Anschluss ist bewusst NICHT Teil dieses Branches (er gehört zum
> Deploy-Branch `wholebodymove`). Erst trainieren, dann anschließen.

---

## 8. Fundierte Parameter-Schätzung (Defaults sind so gesetzt)

Alle Werte sind als **gut-startende Defaults** im Code gesetzt. Begründung + was du
drehst, wenn etwas klemmt:

### Front-Arm-Envelope (`env_cfgs.py`, `ARM_DELTA_LOW/HIGH`)
Deltas **um die Default-Pose** (die selbst schon vorne ist) — garantiert „vorne":
| Gelenk | Default | Range (eff.) | Warum |
|---|---|---|---|
| shoulder_pitch | 0.35 | **0.10 … 0.90** | nur vor/hoch, nie hinter den Körper |
| elbow | 0.87 | **0.42 … 1.42** | immer gebeugt = Tragehaltung |
| shoulder_roll | ±0.18 | ±0.25 Delta | Hände zusammen/leicht auf |
| shoulder_yaw | 0 | ±0.35 | Unterarm vor dem Körper drehen |
| wrists | 0 | ±0.20 | nahezu neutral |
→ Hände bleiben **im Frontbereich**. Willst du höhere/breitere Tragehaltungen,
`shoulder_pitch`-High und `shoulder_roll` aufweiten.

### Curriculum (`arm_pose_levels`)
`scale 0.3 → 0.6 (≈Iter 1500) → 1.0 (≈Iter 4000)`. **Konservativ.** Wenn Stürze
beim Hochschalten hochgehen: Stufen später setzen (z.B. 3000 / 7000) oder
`init_delta_scale` senken. Wenn es zu leicht/langweilig konvergiert: früher.

### Reward
| Term | Wert | Begründung |
|---|---|---|
| `arm_pose_tracking.weight` | **1.0** | gleichrangig zum Velocity-Tracking |
| `arm_pose_tracking.std` | **0.4** rad | bei ~0.1 rad Fehler ≈ 0.94 Reward (sanft, wie `pose`) |
| `pose`/`stand_still` | unverändert, **nur Beine+Taille** | Beinverhalten bleibt wie beim bewährten Walker |
Wenn die Arme der Pose **nicht gut folgen**: `std` auf 0.3 senken **oder** weight auf 1.5–2.0. Wenn die Arme die **Balance stören** (Beine fallen): weight auf 0.5 senken, Envelope langsamer aufweiten.

### Last (`carry_payload`)
Startup-CoM-Offset am Torso `x:0–0.08 m, z:0–0.05 m`. Für eine **leere** Kiste ist
das bewusst klein (die Vorwärts-Armpose dominiert ohnehin). Schwerere Kiste →
Range hochziehen (z.B. x bis 0.15) und ein Curriculum dafür ergänzen.

### Resampling der Armpose
`(2.0, 4.0) s` — die gehaltene Pose wechselt alle 2–4 s. Realistischer Mix aus
„Pose halten" und „Pose wechseln". Für ruhigeres Tragen: erhöhen (4–8 s).

### Sonstiges (unverändert übernommen, gut für G1)
- `num_envs=4096`, `num_steps_per_env=24`, PPO `lr=1e-3` adaptive, `gamma=0.99`,
  `clip=0.2`, MLP (512,256,128), `max_iterations=15000`.
- `push_robot` (±0.5 m/s alle 5–6 s), `foot_friction` 0.3–1.6, `base_com`-Jitter,
  `encoder_bias` — die bestehende Domain-Randomization bleibt aktiv.

---

## 9. Realistische Erwartung

Dieser eine Lauf zielt direkt auf das komplette Szenario:
- **Front-Posen + leere Kiste: hohe Erfolgschance** — die Stock-Policy ist nah dran;
  wir entfernen den Arm-Default-Pull, konditionieren auf die Pose und der dominante
  Effekt (Vorwärts-Armpose) + minimale Last ist gut lernbar.
- **Schwere/volle Kiste (separat, optional):** härter, braucht hochgezogene Last +
  ggf. mehr Tuning.

Plane **mehrere Trainingsläufe** ein — der **Reward-/Curriculum-Tuning**-Teil ist
der unkalkulierbare, nicht das Coden. Falls ein Lauf instabil wird, drehst du an
den in §8 genannten Stellschrauben und startest neu (ggf. Warm-Start von einem
guten Checkpoint).

## 10. Troubleshooting
- **Task nicht in `list_envs`** → Overlay erneut anwenden; prüfen, dass
  `config/g1_boxcarry/__init__.py` existiert und `mdp/__init__.py` die 3 Importe hat.
- **OOM** → `--env.scene.num-envs=2048`.
- **`resolve_matching_names_values`-Fehler** → ein Arm/Bein-Regex matcht nichts;
  Gelenknamen prüfen (`python scripts/list_envs.py` baut das Env und meldet früh).
- **Arme folgen, aber Roboter fällt bei großem Envelope** → Curriculum strecken,
  `arm_pose_tracking.weight` senken, `push_robot` evtl. kurz reduzieren.
