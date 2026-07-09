#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Test des InspireModbusBackend gegen einen Fake-Modbus-TCP-Server.

Der Fake-Server implementiert FC03/FC16 auf einem Register-Dict und emuliert
damit eine Inspire RH56DFTP-2 (Register-Map wie reference/ftp_hand_controller).
Laeuft komplett ohne Hardware und ohne ROS (stdlib + pytest).
"""

import socket
import struct
import threading
import time

import pytest

from g1pilot.manipulation.inspire_ftp import tactile
from g1pilot.manipulation.inspire_ftp.backends import (
    InspireModbusBackend,
    _REG_ANGLE_ACT, _REG_ANGLE_SET, _REG_FORCE_ACT, _REG_FORCE_SET,
    _REG_SPEED_SET,
)
from g1pilot.manipulation.inspire_ftp.model import HandModel
from g1pilot.manipulation.inspire_ftp.modbus_client import ModbusClient


class FakeInspireHand(threading.Thread):
    """Mini-Modbus-TCP-Server: FC03/FC16 auf einem Register-Dict (ein Client)."""

    def __init__(self):
        super().__init__(daemon=True)
        self.regs = {}
        # Plausible Ist-Werte vorbelegen.
        for i in range(6):
            self.regs[_REG_ANGLE_ACT + i] = 900 - 10 * i
            self.regs[_REG_FORCE_ACT + i] = (65535 if i == 5 else 120 + i)  # -1 -> 0
        for zid, _f, _k, reg, rows, cols in tactile.TACTILE_ZONES:
            for j in range(rows * cols):
                self.regs[reg + j] = (reg + j) % 4096
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(2)
        self.port = self.srv.getsockname()[1]
        self._stop = False
        self.write_log = []          # (addr, [values]) je FC16

    def run(self):
        import select
        conns = []
        while not self._stop:
            try:
                r, _, _ = select.select([self.srv] + conns, [], [], 0.05)
            except (OSError, ValueError):
                break
            for s in r:
                if s is self.srv:
                    try:
                        c, _ = self.srv.accept()
                        c.settimeout(0.5)
                        conns.append(c)
                    except OSError:
                        pass
                    continue
                c = s
                try:
                    hdr = c.recv(8)
                except OSError:
                    conns.remove(c)
                    continue
                if not hdr or len(hdr) < 8:
                    conns.remove(c)
                    c.close()
                    continue
                tid, proto, length = struct.unpack('>HHH', hdr[:6])
                unit, fc = hdr[6], hdr[7]
                body = c.recv(length - 2)
                if fc == 0x03:
                    reg, cnt = struct.unpack('>HH', body[:4])
                    data = b''.join(
                        struct.pack('>H', self.regs.get(reg + i, 0) & 0xFFFF)
                        for i in range(cnt))
                    pdu = bytes([0x03, len(data)]) + data
                elif fc == 0x10:
                    reg, cnt, bc = struct.unpack('>HHB', body[:5])
                    vals = [struct.unpack('>H', body[5 + 2 * i:7 + 2 * i])[0]
                            for i in range(cnt)]
                    for i, v in enumerate(vals):
                        self.regs[reg + i] = v
                    self.write_log.append((reg, vals))
                    pdu = struct.pack('>BHH', 0x10, reg, cnt)
                else:
                    pdu = bytes([fc | 0x80, 0x01])
                mbap = struct.pack('>HHH', tid, proto, len(pdu) + 1) + bytes([unit])
                c.sendall(mbap + pdu)

    def stop(self):
        self._stop = True
        self.srv.close()


@pytest.fixture()
def fake_hand():
    srv = FakeInspireHand()
    srv.start()
    yield srv
    srv.stop()


def _wait_until(cond, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_modbus_client_read_write(fake_hand):
    c = ModbusClient("127.0.0.1", fake_hand.port, timeout=1.0)
    assert c.connect()
    vals = c.read_registers(_REG_ANGLE_ACT, 6)
    assert vals == [900, 890, 880, 870, 860, 850]
    assert c.write_registers(_REG_ANGLE_SET, [1000, 0, 500, 250, 65535, 42])
    assert fake_hand.regs[_REG_ANGLE_SET + 4] == 65535
    # Chunking: mehr als 125 Register am Stueck lesen.
    zid, _f, _k, reg, rows, cols = tactile.TACTILE_ZONES[1]   # 12x8 = 96 ... nimm palme
    reg, rows, cols = 3000, 20, 12                            # 240 > 125 -> 2 Chunks
    big = c.read_registers(reg, rows * cols)
    assert big is not None and len(big) == rows * cols
    c.disconnect()


def test_backend_reads_actuals_and_writes_setpoints(fake_hand):
    models = {"left": HandModel("links", "127.0.0.1"),
              "right": HandModel("rechts", "sim://right")}
    backend = InspireModbusBackend(
        publish_fn=lambda names, pos: None,
        hosts={"left": "127.0.0.1", "right": ""},   # nur links montiert
        models=models,
        port=fake_hand.port,
        poll_rate_hz=50.0,
        tactile_every=2,
    )
    try:
        assert list(backend.workers.keys()) == ["left"]

        # Ist-Werte kommen an (angle_act aus den Fake-Registern, -1-Kraft -> 0).
        assert _wait_until(lambda: models["left"].controller_state()["connected"])
        assert _wait_until(
            lambda: models["left"].controller_state()["angle_act"] == [900, 890, 880, 870, 860, 850])
        st = models["left"].controller_state()
        assert st["force_act"][5] == 0.0 and st["force_act"][0] == 120.0

        # Taktil-Zonen werden gelesen (Form + Werte aus den Fake-Registern).
        assert _wait_until(
            lambda: models["left"].viewer_state()["zones"]["kleiner_tip"]
            == [(3000 + j) % 4096 for j in range(9)])

        # Soll-Werte: erst enabled schalten, dann Winkel setzen -> FC16 landet
        # im richtigen Register inkl. -1 -> 0xFFFF.
        models["left"].set_enabled(True)
        models["left"].set_angle(2, 333)
        assert _wait_until(
            lambda: fake_hand.regs.get(_REG_ANGLE_SET + 2) == 333)
        assert fake_hand.regs[_REG_ANGLE_SET + 0] == 0xFFFF   # -1 = halten
        # Speed wurde beim Verbinden initial geschrieben.
        assert fake_hand.regs.get(_REG_SPEED_SET) == 500
        # Force-Aenderung wird geschrieben.
        models["left"].set_force(1, 777)
        assert _wait_until(lambda: fake_hand.regs.get(_REG_FORCE_SET + 1) == 777)

        # Disabled: keine weiteren Winkel-Writes (Original-Semantik).
        models["left"].set_enabled(False)
        time.sleep(0.1)
        n_writes = len(fake_hand.write_log)
        models["left"].angle_set[2] = 111   # direkt setzen (ohne enable)
        time.sleep(0.2)
        angle_writes = [w for w in fake_hand.write_log[n_writes:]
                        if w[0] == _REG_ANGLE_SET]
        assert angle_writes == []
    finally:
        backend.shutdown()


def test_backend_reconnects(fake_hand):
    models = {"left": HandModel("links", "x"), "right": HandModel("rechts", "y")}
    backend = InspireModbusBackend(
        publish_fn=lambda names, pos: None,
        hosts={"left": "127.0.0.1", "right": ""},
        models=models, port=fake_hand.port,
        poll_rate_hz=50.0, tactile_every=0,
    )
    try:
        assert _wait_until(lambda: models["left"].controller_state()["connected"])
        fake_hand.stop()          # Hand "faellt aus"
        assert _wait_until(
            lambda: not models["left"].controller_state()["connected"], timeout=5.0)
    finally:
        backend.shutdown()
