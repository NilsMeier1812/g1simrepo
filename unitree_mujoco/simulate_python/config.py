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
