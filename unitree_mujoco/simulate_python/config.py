import os

ROBOT = "g1" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 0 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

# Sim-Zeitschritt. 1 kHz (0.001) ist noetig, damit der extern gerechnete PD-
# Daempfungsterm (kd) NUMERISCH STABIL bleibt: bei 0.005 schwingen die leichten
# Armgelenke auf (explizite Integration ueberschiesst, mehr kd macht es schlimmer).
# Messung (sim_hold_test): dt=0.005 -> 0.17 rad Zittern, dt=0.002 -> 0.02 rad,
# dt=0.001 -> 0.0001 rad (praktisch null). Bei Realtime-Problemen 0.002 nehmen.
SIMULATE_DT = 0.001
VIEWER_DT = 0.02  # 50 fps for viewer


# === SIM REALTIME FACTOR ===
# Laesst die Sim langsamer als Echtzeit laufen (1.0 = Echtzeit). Auf langsamen
# PCs schafft loco_sim die 50-Hz-Policy-Inferenz nur mit ~30 Hz Wall-Clock. Eine
# auf 50 Hz trainierte Policy bekommt dann pro Sim-Sekunde zu wenige Kommandos
# -> sie ueberschiesst und driftet weg. Loesung: Sim auf z.B. 0.5x bremsen, dann
# vergehen pro Policy-Schritt (~30 Hz) wieder genau ~20 ms PHYSIK -> die Policy
# sieht exakt ihre trainierte 50-Hz-Dynamik (Roboter laeuft optisch in Zeitlupe,
# balanciert aber korrekt). Faustregel: FACTOR <= (gemessene policy-Hz)/50, damit
# die Policy immer mithaelt. Per Env setzbar (run_sim_loco.sh setzt 0.5); Default
# 1.0 laesst die Arm-only-Sim (run_sim.sh) unveraendert.
SIM_REALTIME_FACTOR = float(os.environ.get("SIM_REALTIME_FACTOR", "1.0"))
# === SIM REALTIME FACTOR END ===


# === SIM LOCKSTEP (deterministische Regelrate) ===
# Der SIM_REALTIME_FACTOR-Trick oben ist FRAGIL: er muss pro PC von Hand auf die
# gemessene Wall-Clock-Rate getunt werden, und Timing-Jitter (DDS-Latenz, variable
# Schleifenlast) laesst eine ratenabhaengige RL-Policy chattern/kippen. Lockstep
# loest das an der Wurzel: die Sim macht GARANTIERT exakt SIM_LOCKSTEP_DECIMATION
# Physikschritte pro empfangenem rt/lowcmd und publiziert danach EINEN frischen
# rt/lowstate. Zusammen mit "ein Kommando pro lowstate" im Controller entsteht eine
# strikte 1:1-Alternation -> die 50-Hz-Policy sieht auf JEDEM PC exakt ihre
# trainierten 20 ms Physik/Schritt, voellig unabhaengig von der Wall-Clock.
# SIM_REALTIME_FACTOR wird damit irrelevant (kein Wall-Clock-Sleep im Lockstep).
# Nur im Loco-Modus (HOLD_BASE_MODE=off) sinnvoll; Arm-only/weld bleibt unberuehrt.
#   decimation = control_dt / SIMULATE_DT = 0.02 / 0.001 = 20.
def _env_truthy(name, default=False):
    v = os.environ.get(name)
    if v is None:
        return default
    return str(v).strip().lower() in ("1", "true", "yes", "on")

SIM_LOCKSTEP = _env_truthy("SIM_LOCKSTEP", False)
SIM_CONTROL_DT = float(os.environ.get("SIM_CONTROL_DT", "0.02"))
SIM_LOCKSTEP_DECIMATION = max(1, int(round(SIM_CONTROL_DT / SIMULATE_DT)))
# Watchdog: kommt im Lockstep kein neues Kommando (Controller noch nicht verbunden
# oder abgestuerzt), trotzdem nach diesem Wall-Clock-Limit weiterschreiten, damit
# die Sim/der Viewer nie hart einfriert.
SIM_LOCKSTEP_WATCHDOG_S = float(os.environ.get("SIM_LOCKSTEP_WATCHDOG_S", "0.2"))
# === SIM LOCKSTEP END ===


# === HOLD_BASE FEATURE ===
# Hält den Oberkörper ruhig, damit die Arme ohne Balance-Controller getestet
# werden können. HOLD_BASE_MODE waehlt WIE:
#   "weld"     : torso_link (Basis beider Arme) per Weld-Constraint starr an die
#                Welt + Beine/Taille als steife Federn. Impulserhaltend, KEIN
#                Teleport -> sauberste, jitterfreie Basis fuer die Arme. (Default)
#   "teleport" : Legacy. Becken/Beine/Taille werden jeden Sim-Schritt hart auf
#                qpos0 gesetzt (Weld aus). Kann Zittern in die Arme pumpen.
#   "off"      : Basis voellig frei (Weld aus) - fuer echten Loco-Controller.
# HOLD_BASE (bool) bleibt aus Rueckwaerts-Kompatibilitaet: True -> nutzt
# HOLD_BASE_MODE, False -> erzwingt "off".
# Per Env ueberschreibbar, damit man ohne config-Edit zwischen Arm-only (weld)
# und freiem Loco-Betrieb (off) umschalten kann:
#   HOLD_BASE_MODE=off docker compose ... up   -> Basis frei, Loco regelt die Beine
HOLD_BASE = True
HOLD_BASE_MODE = os.environ.get("HOLD_BASE_MODE", "weld")
# Federwerte fuer Beine/Taille im "weld"-Modus (nur dort genutzt).
HOLD_BASE_STIFFNESS = 2000.0
HOLD_BASE_DAMPING = 80.0
# === HOLD_BASE END ===


# === LOCO STARTUP HOLD ===
# Nur im "off"-Modus (freie Basis, Loco-Betrieb): haelt Beine+Taille (Motor 0..14)
# in einer Standpose, SOLANGE noch KEIN Loco-Controller auf rt/lowcmd kommandiert
# hat. Ueberbrueckt das Startfenster (MuJoCo steppt sofort, der g1pilot-Container
# braucht aber erst colcon build + Launch, bis loco_sim verbunden ist) — sonst
# faellt der Roboter bei freier Basis um, bevor man START BALANCING druecken kann.
# Sobald das erste rt/lowcmd ankommt, uebernimmt der Regler nahtlos (der Hold ist
# dann inaktiv). Pose = Policy-default_angles (leicht gebeugte Knie), damit der
# Uebergang zu loco_sims HOLD-Zustand ohne Sprung passiert.
LOCO_STARTUP_HOLD = True
LOCO_STARTUP_HOLD_POSE = [
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   # linkes Bein:  hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,   # rechtes Bein: dito
    0.0, 0.0, 0.0,                    # Taille: yaw, roll, pitch
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # linker Arm  (15..21): gehalten -> kein Limp-Fall beim Laden
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # rechter Arm (22..28); 0 = loco_sims HOLD-Ziel -> kein Sprung
]
LOCO_STARTUP_HOLD_KP = 100.0
# Kraeftig gedaempft: die im verschweissten Startup-Hold frei haengenden Beine
# (Fuesse knapp ueber dem Boden) sollen NICHT schwingen, sondern ruhig auf der
# Startpose sitzen. kd=2 ist dafuer zu schwach (untergedaempft). Passt zur
# erhoehten HOLD-Daempfung in loco_sim (hold_kd_scale).
LOCO_STARTUP_HOLD_KD = 12.0
# === LOCO STARTUP HOLD END ===


# === LOCO MANAGED WELD ===
# Ein freistehender Zweibeiner laesst sich NICHT rein statisch (ohne aktive Balance)
# aufrecht halten — er kippt in ~1-2 s. Nur die RL-Policy balanciert aktiv. Auf
# langsamen Rechnern faellt der Roboter aber um, bevor loco_sim (nach colcon build +
# Launch) ueberhaupt verbunden ist. Loesung: im off-Modus die Basis (torso_link-Weld)
# GEHALTEN lassen, bis loco_sim das Balancieren startet. loco_sim signalisiert seinen
# Zustand ueber rt/lowcmd motor_cmd[29].q (Bein-Kanal, dort sonst ungenutzt):
#   0 = HOLD (Basis gehalten/Weld an), 1 = RUN (Basis frei + in Stand-Pose stellen),
#   2 = DAMP (Basis frei, kein Reset -> sanftes Hinsetzen).
# Beim Wechsel nach RUN setzt die Bridge den Roboter in eine saubere Stand-Pose
# (Beine = default_angles, aufrecht, v=0) und loest den Weld -> die Policy startet aus
# genau dem Zustand, in dem sie zuverlaessig balanciert. Das macht das Start-Timing
# (langsamer PC) egal und ersetzt das "Operator haelt den Roboter"-Schrittchen des
# echten Roboters.
LOCO_MANAGED_WELD = True
LOCO_RESET_PELVIS_Z = 0.78   # Pelvis-Hoehe beim Aufstehen in die Stand-Pose (Fuesse am Boden)
# === LOCO MANAGED WELD END ===


# === SIM PUSH (Stoer-Impuls / "Schubs") ===
# Stoer-Test fuer den Balancer: ein UDP-Trigger (vom Streamdeck-Button ueber
# loco_sim) bringt fuer kurze Zeit eine externe Kraft in ZUFAELLIGER horizontaler
# Richtung auf den Oberkoerper auf. So sieht man, wie gut der Balancer Stoerungen
# ausgleicht. Details/Protokoll: push_listener.py.
PUSH_ENABLE = _env_truthy("SIM_PUSH_ENABLE", True)
PUSH_UDP_PORT = int(os.environ.get("SIM_PUSH_PORT", "47900"))
PUSH_BODY = os.environ.get("SIM_PUSH_BODY", "torso_link")     # Angriffspunkt
PUSH_FORCE_N = float(os.environ.get("SIM_PUSH_FORCE_N", "150.0"))    # Kraft [N]
PUSH_DURATION_S = float(os.environ.get("SIM_PUSH_DURATION_S", "0.12"))  # Dauer [s]
# Impuls = Kraft x Dauer (150 N x 0.12 s = 18 Ns -> bei ~40 kg ~0.45 m/s Stoss).
# === SIM PUSH END ===
