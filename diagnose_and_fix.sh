#!/bin/bash
set -e
cd ~/g1_pilot_sim

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " PHASE 1/7: Backup aktueller Zustand"
echo "═══════════════════════════════════════════════════════════════"
BACKUP_DIR="/tmp/g1pilot_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
cp unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py "$BACKUP_DIR/"
cp unitree_mujoco/simulate_python/unitree_mujoco.py "$BACKUP_DIR/"
echo "Backup: $BACKUP_DIR"

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " PHASE 2/7: Reparatur bridge.py (SyntaxError beheben)"
echo "═══════════════════════════════════════════════════════════════"
python3 << 'PYEOF'
path = "unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py"
with open(path) as f: content = f.read()

BROKEN = """    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_

# Shared ctrl array written by LowCmdHandler, read by SimulationThread
import numpy as _np
_g_ctrl = None
_g_ctrl_ready = False as LowState_default
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default"""

FIXED = """    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

# Shared ctrl array (Workaround falls bridge.mj_data != sim.mj_data)
import numpy as _np
_g_ctrl = None
_g_ctrl_ready = False"""

if BROKEN in content:
    content = content.replace(BROKEN, FIXED)
    with open(path, "w") as f: f.write(content)
    print("✓ Bridge file repariert")
else:
    print("✗ Broken pattern nicht gefunden — Datei in anderem Zustand")
    print("Zeilen 14-26:")
    for i, l in enumerate(content.split("\n")[13:26], 14):
        print(f"  {i}: {l}")

# Syntax-Check
import ast
try:
    ast.parse(content)
    print("✓ Python syntax OK")
except SyntaxError as e:
    print(f"✗ SyntaxError Zeile {e.lineno}: {e.msg}")
    print(f"    {e.text}")
    exit(1)
PYEOF

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " PHASE 3/7: Diagnose-Hooks in unitree_mujoco.py einbauen"
echo "═══════════════════════════════════════════════════════════════"
python3 << 'PYEOF'
path = "unitree_mujoco/simulate_python/unitree_mujoco.py"
with open(path) as f: content = f.read()

# Hooks direkt nach Bridge-Konstruktion einbauen
OLD = "    unitree = UnitreeSdk2Bridge(mj_model, mj_data)"
NEW = """    unitree = UnitreeSdk2Bridge(mj_model, mj_data)
    import threading as _t
    print(f'[DIAG] tid_sim={_t.get_ident()}', flush=True)
    print(f'[DIAG] SAME_OBJECT={mj_data is unitree.mj_data}', flush=True)
    print(f'[DIAG] outer_id={id(mj_data)} bridge_id={id(unitree.mj_data)}', flush=True)
    print(f'[DIAG] outer_ctrl_ptr={mj_data.ctrl.ctypes.data}', flush=True)
    print(f'[DIAG] bridge_ctrl_ptr={unitree.mj_data.ctrl.ctypes.data}', flush=True)
    print(f'[DIAG] num_motor={mj_model.nu} ctrl_len={len(mj_data.ctrl)}', flush=True)
    print(f'[DIAG] actuator_gaintype={list(mj_model.actuator_gaintype[:5])}...', flush=True)
    print(f'[DIAG] actuator_biastype={list(mj_model.actuator_biastype[:5])}...', flush=True)"""

if "[DIAG] SAME_OBJECT" not in content and OLD in content:
    content = content.replace(OLD, NEW)
    with open(path, "w") as f: f.write(content)
    print("✓ DIAG hooks eingebaut")
elif "[DIAG] SAME_OBJECT" in content:
    print("○ DIAG hooks bereits vorhanden")
else:
    print("✗ Bridge-Konstruktion Zeile nicht gefunden")

# Syntax-Check
import ast
try:
    ast.parse(content)
    print("✓ unitree_mujoco.py syntax OK")
except SyntaxError as e:
    print(f"✗ SyntaxError Zeile {e.lineno}: {e.msg}")
    exit(1)
PYEOF

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " PHASE 4/7: Tid-Diagnose in LowCmdHandler"
echo "═══════════════════════════════════════════════════════════════"
python3 << 'PYEOF'
path = "unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py"
with open(path) as f: content = f.read()

OLD = "        print(f'[CMD] ctrl22={_g_ctrl[22]:.3f}', flush=True)"
NEW = """        import threading as _t
        print(f'[CMD] tid={_t.get_ident()} ctrl22={_g_ctrl[22]:.3f} mj_data_id={id(self.mj_data)} ctrl_ptr={self.mj_data.ctrl.ctypes.data}', flush=True)
        # Direkt-Schreib-Test:
        self.mj_data.ctrl[22] = _g_ctrl[22]
        print(f'[CMD2] nach direkt-schreib: mj_data.ctrl[22]={self.mj_data.ctrl[22]:.3f}', flush=True)"""

if "[CMD2]" not in content and OLD in content:
    content = content.replace(OLD, NEW)
    with open(path, "w") as f: f.write(content)
    print("✓ Erweiterte CMD Diagnose eingebaut")

# SIM-Print muss IMMER drucken (nicht nur wenn != 0)
OLD2 = """        _ctrl22_before = float(mj_data.ctrl[22])
        if _ctrl22_before == 0: print(f'[SIM_PTR] ctrl_ptr={mj_data.ctrl.ctypes.data} val={_ctrl22_before:.3f}', flush=True)"""
NEW2 = """        _ctrl22_before = float(mj_data.ctrl[22])
        # Print 1x pro Sekunde damit wir Pointer + tid sehen
        import threading as _tt
        if not hasattr(SimulationThread, '_last_diag') or (time.time() - SimulationThread._last_diag) > 1.0:
            SimulationThread._last_diag = time.time()
            print(f'[SIM] tid={_tt.get_ident()} mj_data_id={id(mj_data)} ctrl_ptr={mj_data.ctrl.ctypes.data} ctrl22={_ctrl22_before:.3f}', flush=True)"""

path2 = "unitree_mujoco/simulate_python/unitree_mujoco.py"
with open(path2) as f: c2 = f.read()
if "_last_diag" not in c2 and OLD2 in c2:
    c2 = c2.replace(OLD2, NEW2)
    with open(path2, "w") as f: f.write(c2)
    print("✓ Erweiterte SIM Diagnose eingebaut")
PYEOF

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " PHASE 5/7: Container Rebuild"
echo "═══════════════════════════════════════════════════════════════"
cd ~/g1_pilot_sim/g1pilot
make stop 2>/dev/null || true
docker rm -f g1_mujoco_sim g1pilot_sim 2>/dev/null || true
docker build --no-cache -f docker/Dockerfile.mujoco -t g1pilot-mujoco:v1.0 ..

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " PHASE 6/7: Container starten (im Hintergrund)"
echo "═══════════════════════════════════════════════════════════════"
make sim > /tmp/make_sim.log 2>&1 &
echo "Warte 20s auf Container-Start..."
sleep 20

echo "--- Container Status ---"
docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "g1pilot|mujoco|NAMES"

echo ""
echo "--- Erste 60 Zeilen mujoco Container ---"
docker logs g1_mujoco_sim 2>&1 | head -60

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " PHASE 7/7: Test-LowCmd senden und joint[22] beobachten"
echo "═══════════════════════════════════════════════════════════════"
docker exec g1pilot_sim python3 << 'TESTEOF'
import time, threading
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

ChannelFactoryInitialize(1, 'lo')

states = []
def on_state(m):
    states.append((time.time(), round(m.motor_state[22].q, 5)))
sub = ChannelSubscriber('rt/lowstate', LowState_)
sub.Init(on_state, 10)

print("[TEST] Warte 2s auf Subscriber...")
time.sleep(2)
baseline_count = len(states)
print(f"[TEST] Baseline: {baseline_count} Nachrichten empfangen")
print(f"[TEST] joint[22] Baseline: {[s[1] for s in states[-3:]]}")

msg = unitree_hg_msg_dds__LowCmd_()
msg.motor_cmd[22].tau = -25.0
msg.crc = CRC().Crc(msg)
pub = ChannelPublisher('rt/arm_sdk', LowCmd_)
pub.Init()
time.sleep(2)

print("\n[TEST] Sende 50 Befehle @ 30Hz mit tau=-25.0...")
for i in range(50):
    pub.Write(msg)
    time.sleep(0.033)

time.sleep(2)
new_states = states[baseline_count:]
print(f"\n[TEST] Nach Befehlen: {len(new_states)} neue Nachrichten")
if new_states:
    vals = [s[1] for s in new_states]
    print(f"[TEST] joint[22]: min={min(vals):.5f}  max={max(vals):.5f}")
    print(f"[TEST] Erste 5: {vals[:5]}")
    print(f"[TEST] Letzte 5: {vals[-5:]}")
    if max(abs(v) for v in vals) > 0.001:
        print("\n>>> ERFOLG: joint[22] hat sich BEWEGT! <<<")
    else:
        print("\n>>> joint[22] bewegt sich NICHT — siehe Container Logs <<<")
TESTEOF

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " AUSWERTUNG: MuJoCo Container Logs"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "--- DIAG (einmalig beim Start) ---"
docker logs g1_mujoco_sim 2>&1 | grep -E "^\[DIAG\]" | head -10
echo ""
echo "--- CMD (jeder LowCmdHandler-Call) ---"
docker logs g1_mujoco_sim 2>&1 | grep -E "^\[CMD" | head -10
echo ""
echo "--- SIM (1x/sec im SimulationThread) ---"
docker logs g1_mujoco_sim 2>&1 | grep -E "^\[SIM\]" | head -10
echo ""
echo "--- Errors/Tracebacks ---"
docker logs g1_mujoco_sim 2>&1 | grep -iE "error|traceback|exception" | head -10

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " FERTIG. Bitte komplette Ausgabe zurücksenden."
echo "═══════════════════════════════════════════════════════════════"
