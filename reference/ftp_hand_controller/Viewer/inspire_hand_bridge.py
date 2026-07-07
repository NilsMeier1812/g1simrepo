#!/usr/bin/env python3
"""
Inspire RH56DFTP-2 Hand Bridge Server
======================================
Verbindet sich mit linker und rechter Hand via Modbus TCP,
liest Taktilsensor- und Kraft-Daten, und streamt diese
per WebSocket an den HTML-Viewer.

Voraussetzungen:
  pip install pymodbus websockets

sudo ip addr add 192.168.123.100/24 dev enp0s31f6
sudo ip link set enp0s31f6 up

Verwendung:
  python inspire_hand_bridge.py [--left-ip 192.168.123.210] [--right-ip 192.168.123.211]
  
Standard-IPs:
  Linke Hand:  192.168.123.210  Port 6000
  Rechte Hand: 192.168.123.211  Port 6000
  WebSocket:   ws://localhost:8765
"""
#!/usr/bin/env python3

import asyncio, json, argparse, logging, struct, socket, threading, time

try:
    import websockets
except ImportError:
    print("pip install websockets"); exit(1)

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
log = logging.getLogger(__name__)

MODBUS_PORT = 6000
UNIT_ID     = 0xFF

# Direkte Modbus-Register-Adressen (aus Manual + Beispielcode bestätigt)
REG_FORCE_ACT = 1582   # 6x int16, Einheit g, -4000..4000
REG_ANGLE_ACT = 1546   # 6x int16, 0-1000

# Taktil-Register (direkt, NICHT durch 2 geteilt!)
REG_TACTILE_START = 3000   # Kleiner Finger Tip beginnt hier
REG_TACTILE_END   = 5123   # Palme endet hier (inkl.)
REG_TACTILE_COUNT = REG_TACTILE_END - REG_TACTILE_START + 1  # 2124 Register

# Taktil-Zonen (Register-Adressen direkt aus Manual)
TACTILE_ZONES = [
    ("kleiner_tip",  "Kleiner", "tip",  3000,  3,  3),
    ("kleiner_nail", "Kleiner", "nail", 3018, 12,  8),
    ("kleiner_pad",  "Kleiner", "pad",  3210, 10,  8),
    ("ring_tip",     "Ring",    "tip",  3370,  3,  3),
    ("ring_nail",    "Ring",    "nail", 3388, 12,  8),
    ("ring_pad",     "Ring",    "pad",  3580, 10,  8),
    ("mittel_tip",   "Mittel",  "tip",  3740,  3,  3),
    ("mittel_nail",  "Mittel",  "nail", 3758, 12,  8),
    ("mittel_pad",   "Mittel",  "pad",  3950, 10,  8),
    ("zeige_tip",    "Zeige",   "tip",  4110,  3,  3),
    ("zeige_nail",   "Zeige",   "nail", 4128, 12,  8),
    ("zeige_pad",    "Zeige",   "pad",  4320, 10,  8),
    ("daumen_tip",   "Daumen",  "tip",  4480,  3,  3),
    ("daumen_nail",  "Daumen",  "nail", 4498, 12,  8),
    ("daumen_mid",   "Daumen",  "mid",  4690,  3,  3),
    ("daumen_pad",   "Daumen",  "pad",  4708, 12,  8),
    ("palme",        "Palme",   "palm", 4900,  8, 14),
]

FINGER_LAYOUT = {
    "Kleiner": ["kleiner_tip","kleiner_nail","kleiner_pad"],
    "Ring":    ["ring_tip","ring_nail","ring_pad"],
    "Mittel":  ["mittel_tip","mittel_nail","mittel_pad"],
    "Zeige":   ["zeige_tip","zeige_nail","zeige_pad"],
    "Daumen":  ["daumen_tip","daumen_nail","daumen_mid","daumen_pad"],
    "Palme":   ["palme"],
}

DOF_NAMES = ["Kleiner","Ringfinger","Mittelfinger","Zeigefinger","Daumen-Beug.","Daumen-Rot."]


# ── Modbus TCP (ohne externe Bibliothek) ──────────────────────────────────────

class ModbusClient:
    def __init__(self, host, port=MODBUS_PORT, timeout=2.0):
        self.host    = host
        self.port    = port
        self.timeout = timeout
        self.sock    = None
        self._tid    = 0
        self._lock   = threading.Lock()

    def connect(self):
        try:
            s = socket.socket()
            s.settimeout(self.timeout)
            s.connect((self.host, self.port))
            self.sock = s
            log.info(f"✓ Verbunden: {self.host}:{self.port}")
            return True
        except Exception as e:
            log.warning(f"✗ {self.host}: {e}")
            self.sock = None
            return False

    def disconnect(self):
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None

    def is_connected(self):
        return self.sock is not None

    def read_holding_registers(self, address, count):
        """
        Liest 'count' Register ab 'address'.
        Entspricht: client.read_holding_registers(address, count) aus pymodbus.
        Gibt uint16-Liste zurück oder None bei Fehler.
        """
        if not self.sock:
            return None
        result = []
        remaining = count
        offset = 0
        with self._lock:
            while remaining > 0:
                chunk = min(remaining, 125)
                vals  = self._fc03(address + offset, chunk)
                if vals is None:
                    return None
                result.extend(vals)
                offset    += chunk
                remaining -= chunk
        return result

    def _fc03(self, reg, cnt):
        try:
            self._tid = (self._tid + 1) & 0xFFFF
            pdu  = struct.pack('>BHH', 0x03, reg, cnt)
            mbap = struct.pack('>HHH', self._tid, 0, len(pdu) + 1) + bytes([UNIT_ID])
            self.sock.sendall(mbap + pdu)

            hdr = self._recv(9)
            if not hdr or len(hdr) < 9:
                return None
            if hdr[7] == 0x83:
                log.warning(f"Modbus Exception {hdr[8]:#04x} bei Reg {reg}")
                return None
            bc   = hdr[8]
            data = self._recv(bc)
            if not data or len(data) < bc:
                return None
            return [struct.unpack('>H', data[i*2:i*2+2])[0] for i in range(bc // 2)]
        except Exception as e:
            log.debug(f"FC03 Fehler {self.host}: {e}")
            self.sock = None
            return None

    def _recv(self, n):
        buf = b''
        deadline = time.time() + self.timeout
        while len(buf) < n and time.time() < deadline:
            try:
                c = self.sock.recv(n - len(buf))
                if not c:
                    return None
                buf += c
            except socket.timeout:
                break
        return buf if len(buf) == n else None


# ── Hand-Leser ────────────────────────────────────────────────────────────────

def regs_to_le_u12(regs):
    """
    Konvertiert Modbus-Register (Big-Endian uint16) in Little-Endian uint12-Sensorwerte.
    Manual: "16-bit integer, little-endian, low byte first, high byte second, 0-4095"
    Modbus FC03 liest Speicher als Big-Endian: reg = mem[hi]<<8 | mem[lo]
    LE-Wert: mem[lo] | mem[hi]<<8 = (reg & 0xFF)<<8 | (reg>>8)  → Byte-Swap
    """
    result = []
    for r in regs:
        lo = (r >> 8) & 0xFF   # mem[addr]   = LE low byte (Modbus high byte)
        hi = r & 0xFF           # mem[addr+1] = LE high byte (Modbus low byte)
        val = lo | (hi << 8)
        result.append(val & 0x0FFF)
    return result


class HandReader(threading.Thread):
    def __init__(self, name, host):
        super().__init__(daemon=True)
        self.hand_name = name
        self.host      = host
        self.client    = ModbusClient(host)
        self._lock     = threading.Lock()
        self._running  = True
        self._data     = {
            "connected":  False,
            "host":       host,
            "name":       name,
            "force":      [0.0] * 6,
            "angle":      [0]   * 6,
            "zones":      {z[0]: [0]*z[4]*z[5] for z in TACTILE_ZONES},
            "tactile_raw": [],
        }

    def get(self):
        with self._lock:
            return dict(self._data)

    def run(self):
        while self._running:
            if not self.client.is_connected():
                ok = self.client.connect()
                with self._lock:
                    self._data["connected"] = ok
                if not ok:
                    time.sleep(2.0)
                    continue

            ok = self._cycle()
            with self._lock:
                self._data["connected"] = ok
            if not ok:
                self.client.disconnect()
                time.sleep(2.0)
            else:
                time.sleep(0.05)

    def _cycle(self):
        # ── FORCE_ACT ────────────────────────────────────────────────────
        regs = self.client.read_holding_registers(REG_FORCE_ACT, 6)
        if regs is None:
            return False
        force = []
        for v in regs:
            s = v if v < 32768 else v - 65536
            force.append(0.0 if s == -1 else round(abs(s) / 100.0, 1))

        # ── ANGLE_ACT ────────────────────────────────────────────────────
        regs = self.client.read_holding_registers(REG_ANGLE_ACT, 6)
        angle = [0] * 6
        if regs:
            angle = [0 if v == 0xFFFF else (v if v < 32768 else v - 65536) for v in regs]

        # ── Taktil: alle Zonen einlesen ───────────────────────────────────
        zones     = {}
        all_raw   = []

        for zone_id, _, _, reg_addr, rows, cols in TACTILE_ZONES:
            n    = rows * cols
            regs = self.client.read_holding_registers(reg_addr, n)
            if regs is None:
                with self._lock:
                    zones[zone_id] = self._data["zones"].get(zone_id, [0]*n)
                all_raw.extend([0]*n)
                continue

            # LE uint12 Konvertierung
            vals = regs_to_le_u12(regs)
            zones[zone_id] = vals
            all_raw.extend(vals)

        with self._lock:
            self._data["force"]       = force
            self._data["angle"]       = angle
            self._data["zones"]       = zones
            self._data["tactile_raw"] = all_raw
        return True


# ── WebSocket-Server ──────────────────────────────────────────────────────────

class Bridge:
    def __init__(self, left_ip, right_ip, ws_port=8765):
        self.left    = HandReader("links",  left_ip)
        self.right   = HandReader("rechts", right_ip)
        self.ws_port = ws_port
        self.clients = set()

    async def handler(self, ws):
        self.clients.add(ws)
        log.info(f"Viewer verbunden")
        # Metadaten senden
        meta = {
            "type":    "meta",
            "zones":   [{"id":z[0],"finger":z[1],"zone":z[2],"rows":z[4],"cols":z[5]}
                        for z in TACTILE_ZONES],
            "fingers": FINGER_LAYOUT,
            "dof_names": DOF_NAMES,
        }
        try:
            await ws.send(json.dumps(meta))
            async for _ in ws:
                pass
        except:
            pass
        finally:
            self.clients.discard(ws)

    async def broadcast(self):
        while True:
            if self.clients:
                left  = self.left.get()
                right = self.right.get()
                msg = json.dumps({
                    "type":  "data",
                    "left":  left,
                    "right": right,
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
        log.info(f"Bridge läuft – ws://localhost:{self.ws_port}")
        log.info(f"  Links:  {self.left.host}")
        log.info(f"  Rechts: {self.right.host}")
        async with websockets.serve(self.handler, "localhost", self.ws_port):
            await self.broadcast()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--left-ip",  default="192.168.123.210")
    p.add_argument("--right-ip", default="192.168.123.211")
    p.add_argument("--ws-port",  default=8765, type=int)
    a = p.parse_args()
    try:
        asyncio.run(Bridge(a.left_ip, a.right_ip, a.ws_port).run())
    except KeyboardInterrupt:
        print("Bridge beendet.")