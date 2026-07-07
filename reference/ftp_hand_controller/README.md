# Referenz: originaler ftp_hand_controller (echte Hardware, Modbus TCP)

Die beiden eigenstaendigen Skripte, die VOR der Sim-Integration die echten
Inspire RH56DFTP-2 Haende per **Modbus TCP** angesteuert haben (vom Branch
`reference/ftp_hand_controll` uebernommen, Commit 78337ea). Sie werden hier
**nicht ausgefuehrt** — sie sind die verbindliche Referenz fuer das
`InspireModbusBackend` in `g1pilot/g1pilot/manipulation/inspire_ftp/`.

| Datei | Zweck |
|-------|-------|
| `Controller/hand_controller_bridge.py` | Soll-Winkel/Kraft/Speed schreiben + Ist lesen (Register-Map!) |
| `Viewer/inspire_hand_bridge.py`        | Taktil-Zonen + Kraefte lesen (Taktil-Register-Map!) |
| `*/*.html`                             | Urspruengliche GUIs (die Repo-GUIs sind daraus weiterentwickelt) |

## Verbindliche Hardware-Fakten (aus diesen Skripten)

- **IPs/Port:** links `192.168.123.210`, rechts `192.168.123.211`, Port `6000`,
  Modbus-Unit-ID `0xFF`.
- **Register (direkte Adressen, NICHT durch 2 teilen):**
  `angleSet=1486`, `forceSet=1498`, `speedSet=1522`, `angleAct=1546`,
  `forceAct=1582` (je 6x int16). Taktil: Register 3000–5123 (Zonen-Tabelle in
  `g1pilot/.../inspire_ftp/tactile.py`, dort bereits uebernommen).
- **Konventionen:** Winkel 0..1000 (1000=offen), `-1` = halten (als `0xFFFF`
  schreiben); forceAct in Gramm (-4000..4000, `-1` als 0 werten);
  FC03 lesen (max. 125 Register/Chunk), FC16 schreiben; beim Verbinden
  zuerst speedSet schreiben. close_hand: Daumen-Beugung auf 200 (nicht 0).

Die Original-README verwies noch auf zwei Screenshots (`image.png`,
`image-1.png`) — die sind hier weggelassen (nur Bilder der GUIs).
