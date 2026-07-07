#!/usr/bin/env python3
"""
Inspire RH56DFTP-2 – Hand Controller Bridge
============================================
Liest Ist-Werte (angleAct, forceAct) und schreibt Soll-Werte
(angleSet, forceSet, speedSet) via Modbus TCP.

Register-Adressen (direkte Modbus-Adressen, NICHT durch 2 teilen!):
  angleSet  = 1486   6× int16, 0-1000 (-1 = keine Aktion)
  forceSet  = 1498   6× int16, 0-3000 (g)
  speedSet  = 1522   6× int16, 0-1000
  angleAct  = 1546   6× int16, 0-1000
  forceAct  = 1582   6× int16, -4000..4000 (g)

pip install websockets
python hand_controller_bridge.py --left-ip 192.168.123.210 --right-ip 192.168.123.211
"""

import asyncio, json, argparse, logging, struct, socket, threading, time

try:
    import websockets
except ImportError:
    print("pip install websockets"); exit(1)

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
log = logging.getLogger(__name__)

MODBUS_PORT = 6000
UNIT_ID     = 0xFF

# Register-Adressen
REG_ANGLE_SET = 1486
REG_FORCE_SET = 1498
REG_SPEED_SET = 1522
REG_ANGLE_ACT = 1546
REG_FORCE_ACT = 1582

DOF_NAMES = ["Kleiner", "Ringfinger", "Mittelfinger", "Zeigefinger", "Daumen-Beug.", "Daumen-Rot."]


# ── Modbus Client ─────────────────────────────────────────────────────────────

class ModbusClient:
    def __init__(self, host, port=MODBUS_PORT, timeout=2.0):
        self.host = host; self.port = port; self.timeout = timeout
        self.sock = None; self._tid = 0; self._lock = threading.Lock()

    def connect(self):
        try:
            s = socket.socket(); s.settimeout(self.timeout)
            s.connect((self.host, self.port))
            self.sock = s
            log.info(f"✓ {self.host}:{self.port}")
            return True
        except Exception as e:
            log.warning(f"✗ {self.host}: {e}"); self.sock = None; return False

    def disconnect(self):
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None

    def is_connected(self): return self.sock is not None

    def read_registers(self, addr, count):
        if not self.sock: return None
        result = []; rem = count; off = 0
        with self._lock:
            while rem > 0:
                chunk = min(rem, 125)
                v = self._fc03(addr + off, chunk)
                if v is None: return None
                result.extend(v); off += chunk; rem -= chunk
        return result

    def write_registers(self, addr, values):
        """FC16: schreibt mehrere Register."""
        if not self.sock: return False
        with self._lock:
            return self._fc16(addr, values)

    def _fc03(self, reg, cnt):
        try:
            self._tid = (self._tid + 1) & 0xFFFF
            pdu  = struct.pack('>BHH', 0x03, reg, cnt)
            mbap = struct.pack('>HHH', self._tid, 0, len(pdu)+1) + bytes([UNIT_ID])
            self.sock.sendall(mbap + pdu)
            h = self._recv(9)
            if not h or len(h)<9 or h[7]==0x83: return None
            d = self._recv(h[8])
            if not d: return None
            return [struct.unpack('>H', d[i*2:i*2+2])[0] for i in range(h[8]//2)]
        except Exception as e:
            log.debug(f"FC03 {self.host}: {e}"); self.sock = None; return None

    def _fc16(self, reg, values):
        try:
            self._tid = (self._tid + 1) & 0xFFFF
            data = b''.join(struct.pack('>H', v & 0xFFFF) for v in values)
            pdu  = struct.pack('>BHHB', 0x10, reg, len(values), len(data)) + data
            mbap = struct.pack('>HHH', self._tid, 0, len(pdu)+1) + bytes([UNIT_ID])
            self.sock.sendall(mbap + pdu)
            h = self._recv(12)  # FC16 response = 12 bytes
            return h is not None and len(h) >= 6 and h[7] == 0x10
        except Exception as e:
            log.debug(f"FC16 {self.host}: {e}"); self.sock = None; return False

    def _recv(self, n):
        buf = b''; dl = time.time() + self.timeout
        while len(buf) < n and time.time() < dl:
            try:
                c = self.sock.recv(n - len(buf))
                if not c: return None
                buf += c
            except socket.timeout: break
        return buf if len(buf) == n else None


# ── Hand Controller ───────────────────────────────────────────────────────────

class HandController(threading.Thread):
    def __init__(self, name, host):
        super().__init__(daemon=True)
        self.hand_name = name
        self.host = host
        self.client = ModbusClient(host)
        self._lock = threading.Lock()
        self._running = True

        # Soll-Werte (werden vom Viewer gesetzt)
        self._angle_set = [-1, -1, -1, -1, -1, -1]   # -1 = keine Aktion
        self._force_set = [500, 500, 500, 500, 500, 500]
        self._speed_set = [500, 500, 500, 500, 500, 500]
        self._enabled   = False   # Hauptschalter
        self._pending_write = False

        # Ist-Werte (werden von der Hand gelesen)
        self._angle_act = [0] * 6
        self._force_act = [0] * 6
        self._connected = False

    # ── Setter (vom WebSocket aufgerufen) ────────────────────────────────────

    def set_angle(self, idx, val):
        with self._lock:
            self._angle_set[idx] = int(val)
            self._pending_write = True

    def set_force(self, idx, val):
        with self._lock:
            self._force_set[idx] = int(val)
            self._pending_write = True

    def set_speed_all(self, val):
        with self._lock:
            self._speed_set = [int(val)] * 6
            self._pending_write = True

    def set_enabled(self, enabled):
        with self._lock:
            self._enabled = bool(enabled)
            if not enabled:
                # Hand öffnen: alle Winkel auf -1 (keine Bewegung) oder 0
                self._angle_set = [-1, -1, -1, -1, -1, -1]
            self._pending_write = True

    def get_state(self):
        with self._lock:
            return {
                "connected":  self._connected,
                "host":       self.host,
                "name":       self.hand_name,
                "angle_act":  list(self._angle_act),
                "force_act":  list(self._force_act),
                "angle_set":  list(self._angle_set),
                "force_set":  list(self._force_set),
                "speed_set":  self._speed_set[0],
                "enabled":    self._enabled,
            }

    # ── Thread ────────────────────────────────────────────────────────────────

    def run(self):
        while self._running:
            if not self.client.is_connected():
                ok = self.client.connect()
                with self._lock: self._connected = ok
                if not ok: time.sleep(2.0); continue
                # Beim Verbinden: Speed schreiben
                with self._lock:
                    speed = list(self._speed_set)
                self.client.write_registers(REG_SPEED_SET, speed)

            ok = self._cycle()
            with self._lock: self._connected = ok
            if not ok:
                self.client.disconnect(); time.sleep(2.0)
            else:
                time.sleep(0.05)

    def _cycle(self):
        # Ist-Werte lesen
        av = self.client.read_registers(REG_ANGLE_ACT, 6)
        if av is None: return False
        fv = self.client.read_registers(REG_FORCE_ACT, 6)
        if fv is None: return False

        angle_act = [v if v < 32768 else v - 65536 for v in av]
        force_act = []
        for v in fv:
            s = v if v < 32768 else v - 65536
            force_act.append(0 if s == -1 else s)

        with self._lock:
            self._angle_act = angle_act
            self._force_act = force_act
            do_write   = self._pending_write and self._enabled
            angle_set  = list(self._angle_set)
            force_set  = list(self._force_set)
            speed_set  = list(self._speed_set)
            if do_write:
                self._pending_write = False

        # Soll-Werte schreiben wenn nötig
        if do_write:
            # Speed zuerst
            self.client.write_registers(REG_SPEED_SET, speed_set)
            # Force
            self.client.write_registers(REG_FORCE_SET, force_set)
            # Angle (-1 → 0xFFFF)
            angle_reg = [a & 0xFFFF for a in angle_set]
            self.client.write_registers(REG_ANGLE_SET, angle_reg)

        return True


# ── WebSocket Server ──────────────────────────────────────────────────────────

class ControllerBridge:
    def __init__(self, left_ip, right_ip, ws_port=8766):
        self.left    = HandController("links",  left_ip)
        self.right   = HandController("rechts", right_ip)
        self.ws_port = ws_port
        self.clients = set()

    async def handler(self, ws):
        self.clients.add(ws)
        log.info(f"Controller verbunden")
        # Initiale Konfiguration senden
        await ws.send(json.dumps({
            "type": "config",
            "dof_names": DOF_NAMES,
            "left":  self.left.get_state(),
            "right": self.right.get_state(),
        }))
        try:
            async for msg in ws:
                try:
                    cmd = json.loads(msg)
                    self._handle_cmd(cmd)
                except Exception as e:
                    log.warning(f"CMD Fehler: {e}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)

    def _handle_cmd(self, cmd):
        t = cmd.get("type")
        side = cmd.get("side", "left")
        hand = self.left if side == "left" else self.right

        if t == "set_angle":
            hand.set_angle(cmd["dof"], cmd["value"])
        elif t == "set_force":
            hand.set_force(cmd["dof"], cmd["value"])
        elif t == "set_speed":
            hand.set_speed_all(cmd["value"])
            # Beide Hände gleich schnell
            self.left.set_speed_all(cmd["value"])
            self.right.set_speed_all(cmd["value"])
        elif t == "set_enabled":
            hand.set_enabled(cmd["value"])
        elif t == "set_all_angles":
            for i, v in enumerate(cmd["values"]):
                hand.set_angle(i, v)
        elif t == "open_hand":
            for i in range(6):
                hand.set_angle(i, 1000)
        elif t == "close_hand":
            for i in range(6):
                if i == 4:  # Daumen-Beugung
                    hand.set_angle(i, 200)
                    continue
                hand.set_angle(i, 0)

    async def broadcast(self):
        while True:
            if self.clients:
                msg = json.dumps({
                    "type":  "state",
                    "left":  self.left.get_state(),
                    "right": self.right.get_state(),
                })
                dead = set()
                for ws in list(self.clients):
                    try: await ws.send(msg)
                    except: dead.add(ws)
                self.clients -= dead
            await asyncio.sleep(0.05)

    async def run(self):
        self.left.start()
        self.right.start()
        log.info(f"Controller Bridge – ws://localhost:{self.ws_port}")
        async with websockets.serve(self.handler, "localhost", self.ws_port):
            await self.broadcast()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--left-ip",  default="192.168.123.210")
    p.add_argument("--right-ip", default="192.168.123.211")
    p.add_argument("--ws-port",  default=8766, type=int)
    a = p.parse_args()
    try:
        asyncio.run(ControllerBridge(a.left_ip, a.right_ip, a.ws_port).run())
    except KeyboardInterrupt:
        print("Bridge beendet.")