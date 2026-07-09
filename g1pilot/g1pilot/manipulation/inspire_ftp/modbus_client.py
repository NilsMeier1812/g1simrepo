#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
modbus_client.py — Minimaler Modbus-TCP-Client fuer die Inspire RH56DFTP-2
===========================================================================
1:1 aus dem original ftp_hand_controller uebernommen (reference/
ftp_hand_controller/), der mit genau diesen Haenden auf echter Hardware lief.
Bewusst OHNE pymodbus: nur stdlib (socket/struct), keine API-Brueche zwischen
pymodbus 2.x/3.x, und die Eigenheiten der Hand (Unit-ID 0xFF, direkte
Register-Adressen) sind hier erprobt.

Funktionscodes: FC03 (Read Holding Registers, max. 125/Chunk) und
FC16 (Write Multiple Registers). Alle Werte uint16; Vorzeichen-Konvertierung
(int16) macht der Aufrufer.
"""

from __future__ import annotations

import socket
import struct
import threading
import time
from typing import List, Optional

MODBUS_PORT = 6000
UNIT_ID = 0xFF


class ModbusClient:
    """Ein TCP-Socket pro Hand. Nicht threadsicher ueber Instanzen hinweg,
    aber alle Register-Zugriffe einer Instanz sind per Lock serialisiert."""

    def __init__(self, host: str, port: int = MODBUS_PORT, timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self._tid = 0
        self._lock = threading.Lock()

    # ── Verbindung ────────────────────────────────────────────────────────
    def connect(self) -> bool:
        try:
            s = socket.socket()
            s.settimeout(self.timeout)
            s.connect((self.host, self.port))
            self.sock = s
            return True
        except Exception:
            self.sock = None
            return False

    def disconnect(self) -> None:
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def is_connected(self) -> bool:
        return self.sock is not None

    # ── Register-Zugriff ──────────────────────────────────────────────────
    def read_registers(self, addr: int, count: int) -> Optional[List[int]]:
        """FC03, automatisch in 125er-Chunks. uint16-Liste oder None."""
        if not self.sock:
            return None
        result: List[int] = []
        remaining, offset = count, 0
        with self._lock:
            while remaining > 0:
                chunk = min(remaining, 125)
                vals = self._fc03(addr + offset, chunk)
                if vals is None:
                    return None
                result.extend(vals)
                offset += chunk
                remaining -= chunk
        return result

    def write_registers(self, addr: int, values: List[int]) -> bool:
        """FC16, automatisch in 123er-Chunks (Modbus-Limit fuer Writes)."""
        if not self.sock:
            return False
        with self._lock:
            offset = 0
            while offset < len(values):
                chunk = values[offset:offset + 123]
                if not self._fc16(addr + offset, chunk):
                    return False
                offset += len(chunk)
            return True

    # ── Low-Level (MBAP + PDU) ────────────────────────────────────────────
    def _fc03(self, reg: int, cnt: int) -> Optional[List[int]]:
        try:
            self._tid = (self._tid + 1) & 0xFFFF
            pdu = struct.pack('>BHH', 0x03, reg, cnt)
            mbap = struct.pack('>HHH', self._tid, 0, len(pdu) + 1) + bytes([UNIT_ID])
            self.sock.sendall(mbap + pdu)
            h = self._recv(9)
            if not h or len(h) < 9 or h[7] == 0x83:
                return None
            d = self._recv(h[8])
            if not d:
                return None
            return [struct.unpack('>H', d[i * 2:i * 2 + 2])[0] for i in range(h[8] // 2)]
        except Exception:
            self.sock = None
            return None

    def _fc16(self, reg: int, values: List[int]) -> bool:
        try:
            self._tid = (self._tid + 1) & 0xFFFF
            data = b''.join(struct.pack('>H', v & 0xFFFF) for v in values)
            pdu = struct.pack('>BHHB', 0x10, reg, len(values), len(data)) + data
            mbap = struct.pack('>HHH', self._tid, 0, len(pdu) + 1) + bytes([UNIT_ID])
            self.sock.sendall(mbap + pdu)
            h = self._recv(12)  # FC16-Antwort = 12 Bytes
            return h is not None and len(h) >= 8 and h[7] == 0x10
        except Exception:
            self.sock = None
            return False

    def _recv(self, n: int) -> Optional[bytes]:
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


def to_int16(v: int) -> int:
    """uint16 -> int16 (die Hand liefert z.B. forceAct vorzeichenbehaftet)."""
    return v - 65536 if v >= 32768 else v
