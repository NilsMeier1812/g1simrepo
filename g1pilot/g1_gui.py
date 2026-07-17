#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
#  g1_gui.py — Grafisches Start-Menue fuer den G1-Stack (Sim / Real / Szenen).
#
#  Ziel: Im normalen Betrieb muss man KEIN cmd-Fenster mehr anfassen. Dieses
#  Tkinter-Fenster ersetzt die Text-Menues aus:
#     - g1pilot/start.sh                 (Sim / Real starten)
#     - unitree_mujoco/scene_editor/launch.sh   (Umgebungen bearbeiten)
#
#  Es implementiert die Orchestrierung NICHT neu, sondern ruft die bewaehrten
#  Skripte env-getrieben auf:
#     Sim / Real  ->  start.sh --yes           (alle Optionen als Env-Vars)
#     Szenen      ->  scene_editor/launch.sh <cmd> [datei]
#  Die Ausgabe der Skripte (Docker/Compose/Viewer) landet live in einem
#  In-App-Log-Fenster mit Stop-Button — kein Terminal noetig.
#
#  Start:  python3 g1_gui.py        (start.sh startet das automatisch)
#
#  Faellt Tkinter/DISPLAY aus, benutzt start.sh weiter das Text-Menue.
# ════════════════════════════════════════════════════════════════════════
from __future__ import annotations

import os
import queue
import shutil
import subprocess
import sys
import threading
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, simpledialog
except Exception as exc:  # pragma: no cover - nur wenn tkinter fehlt
    sys.stderr.write(
        "[g1_gui] Tkinter ist nicht verfuegbar (%s).\n"
        "         Bitte das klassische Text-Menue nutzen: ./start.sh --menu\n" % exc
    )
    sys.exit(2)


# ── Pfade (alles relativ zu diesem Skript, damit der Aufrufort egal ist) ──
HERE = Path(__file__).resolve().parent                       # .../g1pilot
REPO_ROOT = HERE.parent                                      # Repo-Wurzel
START_SH = HERE / "start.sh"
SCENE_DIR = REPO_ROOT / "unitree_mujoco" / "scene_editor"
SCENES_DIR = SCENE_DIR / "scenes"
LAUNCH_SH = SCENE_DIR / "launch.sh"
SETUP_SH = SCENE_DIR / "setup.sh"
SCENE_VENV = SCENE_DIR / ".venv" / "bin" / "python"

# Dokumente, die aus dem Menue heraus geoeffnet werden koennen.
DOCS = [
    ("README (G1Pilot)", HERE / "README.md"),
    ("Sim testen", HERE / "TESTING_SIM.md"),
    ("Realer Roboter — Checkliste", HERE / "REAL_TESTING.md"),
    ("Navigation", HERE / "NAVIGATION.md"),
    ("Preflight", HERE / "PREFLIGHT.md"),
]

# ── Farb-/Stilkonstanten ────────────────────────────────────────────────
BG = "#1f232b"
CARD = "#2a2f3a"
FG = "#e8eaed"
MUTED = "#9aa0aa"
ACCENT = "#4c8bf5"
GREEN = "#3ecf7a"
RED = "#e5484d"
AMBER = "#e0a13b"


# ════════════════════════════════════════════════════════════════════════
#  Hilfsfunktionen (Host-Umgebung abfragen)
# ════════════════════════════════════════════════════════════════════════
def have(cmd: str) -> bool:
    return shutil.which(cmd) is not None


def docker_ready() -> bool:
    """True, wenn 'docker' existiert und der Daemon erreichbar ist."""
    if not have("docker"):
        return False
    try:
        r = subprocess.run(
            ["docker", "info"], capture_output=True, timeout=6
        )
        return r.returncode == 0
    except Exception:
        return False


def list_scenes() -> list[Path]:
    """Alle Umgebungen aus scene_editor/scenes/*.xml (sortiert)."""
    if not SCENES_DIR.is_dir():
        return []
    return sorted(p for p in SCENES_DIR.glob("*.xml") if p.is_file())


def scene_editor_ready() -> bool:
    """True, wenn das scene_editor-virtualenv eingerichtet ist."""
    return SCENE_VENV.exists()


def detect_nics() -> list[tuple[str, str]]:
    """Physische Netzwerk-Interfaces + IPv4 ermitteln (wie start.sh, in Python).

    Rueckgabe: Liste (name, ip). Bei fehlendem 'ip'-Tool leere Liste.
    """
    nics: list[tuple[str, str]] = []
    if not have("ip"):
        return nics
    try:
        out = subprocess.run(
            ["ip", "-o", "link", "show"], capture_output=True, text=True, timeout=5
        ).stdout
    except Exception:
        return nics
    for line in out.splitlines():
        # Format: "2: eth0: <BROADCAST,...> ..."
        parts = line.split(": ")
        if len(parts) < 2:
            continue
        name = parts[1].split("@")[0].strip()
        if not name or name == "lo":
            continue
        if name.startswith(("docker", "veth", "br-", "virbr")):
            continue
        ip = ""
        try:
            addr = subprocess.run(
                ["ip", "-4", "-o", "addr", "show", name],
                capture_output=True, text=True, timeout=5,
            ).stdout
            for al in addr.splitlines():
                toks = al.split()
                if "inet" in toks:
                    ip = toks[toks.index("inet") + 1]
                    break
        except Exception:
            pass
        nics.append((name, ip))
    return nics


def open_path(path: Path) -> None:
    """Datei mit dem System-Standard oeffnen (Doku etc.)."""
    try:
        if sys.platform == "darwin":
            subprocess.Popen(["open", str(path)])
        elif os.name == "nt":
            os.startfile(str(path))  # type: ignore[attr-defined]
        else:
            subprocess.Popen(["xdg-open", str(path)])
    except Exception as exc:
        messagebox.showerror("Oeffnen fehlgeschlagen", f"{path}\n\n{exc}")


# ════════════════════════════════════════════════════════════════════════
#  Log-Konsole: startet ein Kommando und streamt die Ausgabe live ins Fenster
# ════════════════════════════════════════════════════════════════════════
class ProcessConsole(tk.Toplevel):
    """Ein Fenster, das einen Subprozess ausfuehrt und dessen Ausgabe zeigt.

    - Stdout/Stderr werden zusammengefuehrt und live angezeigt.
    - 'Stoppen' beendet den Prozess (SIGTERM) und ruft optional ein
      Aufraeum-Kommando auf (z.B. 'docker compose down').
    - Thread-sicher via Queue + after()-Polling (Tkinter ist nicht threadsafe).
    """

    def __init__(self, master, title: str, argv: list[str], *,
                 cwd: Path, env: dict | None = None,
                 stop_cmd: list[str] | None = None,
                 stop_note: str = ""):
        super().__init__(master)
        self.title(title)
        self.configure(bg=BG)
        self.geometry("880x560")
        self.minsize(560, 340)

        self._argv = argv
        self._cwd = cwd
        self._env = env
        self._stop_cmd = stop_cmd
        self._stop_note = stop_note
        self._proc: subprocess.Popen | None = None
        self._queue: queue.Queue[str | None] = queue.Queue()
        self._stopping = False

        # Kopfzeile mit dem konkreten Kommando (Transparenz).
        head = tk.Frame(self, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=title, bg=CARD, fg=FG,
                 font=("TkDefaultFont", 11, "bold")).pack(side="left", padx=12, pady=8)
        self._status = tk.Label(head, text="laeuft…", bg=CARD, fg=AMBER,
                                font=("TkDefaultFont", 10, "bold"))
        self._status.pack(side="right", padx=12)

        # Log-Textfeld.
        wrap = tk.Frame(self, bg=BG)
        wrap.pack(fill="both", expand=True, padx=10, pady=(8, 4))
        self._text = tk.Text(wrap, bg="#12151b", fg="#d7dbe0", insertbackground=FG,
                             wrap="none", relief="flat", font=("TkFixedFont", 9),
                             state="disabled")
        yscroll = ttk.Scrollbar(wrap, orient="vertical", command=self._text.yview)
        xscroll = ttk.Scrollbar(self, orient="horizontal", command=self._text.xview)
        self._text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self._text.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        wrap.rowconfigure(0, weight=1)
        wrap.columnconfigure(0, weight=1)
        xscroll.pack(fill="x", padx=10)

        # Fussleiste mit Buttons.
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", padx=10, pady=8)
        self._stop_btn = tk.Button(foot, text="■  Stoppen", command=self.stop,
                                   bg=RED, fg="white", activebackground="#c93b3f",
                                   relief="flat", font=("TkDefaultFont", 10, "bold"),
                                   padx=14, pady=6)
        self._stop_btn.pack(side="left")
        self._close_btn = tk.Button(foot, text="Fenster schliessen",
                                    command=self._on_close, bg=CARD, fg=FG,
                                    relief="flat", padx=12, pady=6)
        self._close_btn.pack(side="right")

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self._append(f"$ {' '.join(argv)}\n\n")
        self._start()
        self.after(80, self._drain)

    # ── intern ──────────────────────────────────────────────────────────
    def _start(self) -> None:
        try:
            self._proc = subprocess.Popen(
                self._argv, cwd=str(self._cwd), env=self._env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                bufsize=1, text=True,
            )
        except Exception as exc:
            self._append(f"[Fehler beim Start] {exc}\n")
            self._set_done(-1)
            return
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        assert self._proc and self._proc.stdout
        for line in self._proc.stdout:
            self._queue.put(line)
        self._proc.wait()
        self._queue.put(None)  # Sentinel: Prozess fertig

    def _drain(self) -> None:
        try:
            while True:
                item = self._queue.get_nowait()
                if item is None:
                    rc = self._proc.returncode if self._proc else -1
                    self._set_done(rc)
                    return
                self._append(item)
        except queue.Empty:
            pass
        self.after(80, self._drain)

    def _append(self, text: str) -> None:
        self._text.configure(state="normal")
        self._text.insert("end", text)
        self._text.see("end")
        self._text.configure(state="disabled")

    def _set_done(self, rc: int) -> None:
        if self._stopping:
            self._status.configure(text="gestoppt", fg=MUTED)
        elif rc == 0:
            self._status.configure(text="beendet (ok)", fg=GREEN)
        else:
            self._status.configure(text=f"beendet (Code {rc})", fg=RED)
        self._stop_btn.configure(state="disabled")

    def stop(self) -> None:
        if self._stopping:
            return
        self._stopping = True
        self._status.configure(text="stoppe…", fg=AMBER)
        self._append("\n[Stoppen angefordert]\n")

        def worker():
            if self._proc and self._proc.poll() is None:
                try:
                    self._proc.terminate()
                    self._proc.wait(timeout=12)
                except Exception:
                    try:
                        self._proc.kill()
                    except Exception:
                        pass
            if self._stop_cmd:
                self._queue.put(f"\n$ {' '.join(self._stop_cmd)}\n")
                try:
                    r = subprocess.run(self._stop_cmd, cwd=str(self._cwd),
                                       capture_output=True, text=True, timeout=90)
                    self._queue.put(r.stdout + r.stderr)
                except Exception as exc:
                    self._queue.put(f"[down-Fehler] {exc}\n")
            self._queue.put(None)

        threading.Thread(target=worker, daemon=True).start()

    def _on_close(self) -> None:
        if self._proc and self._proc.poll() is None:
            if not messagebox.askyesno(
                "Laeuft noch",
                "Der Prozess laeuft noch. Erst stoppen und dann schliessen?"
                + (f"\n\n{self._stop_note}" if self._stop_note else ""),
                parent=self,
            ):
                return
            self.stop()
        self.destroy()


# ════════════════════════════════════════════════════════════════════════
#  Wiederverwendbare Widgets fuer die Options-Dialoge
# ════════════════════════════════════════════════════════════════════════
def section(parent, text: str) -> tk.Frame:
    """Ein abgesetzter 'Card'-Rahmen mit Ueberschrift."""
    outer = tk.Frame(parent, bg=CARD, bd=0, highlightthickness=0)
    outer.pack(fill="x", padx=14, pady=6)
    tk.Label(outer, text=text, bg=CARD, fg=ACCENT,
             font=("TkDefaultFont", 10, "bold")).pack(anchor="w", padx=12, pady=(8, 2))
    inner = tk.Frame(outer, bg=CARD)
    inner.pack(fill="x", padx=12, pady=(0, 10))
    return inner


def toggle_row(parent, label: str, var: tk.BooleanVar, hint: str = "") -> tk.Frame:
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=2)
    cb = tk.Checkbutton(row, text=label, variable=var, bg=CARD, fg=FG,
                        selectcolor=BG, activebackground=CARD, activeforeground=FG,
                        anchor="w", font=("TkDefaultFont", 10))
    cb.pack(side="left")
    if hint:
        tk.Label(row, text=hint, bg=CARD, fg=MUTED,
                 font=("TkDefaultFont", 9)).pack(side="left", padx=8)
    return row


def field_row(parent, label: str, var: tk.StringVar, width: int = 18) -> tk.Entry:
    row = tk.Frame(parent, bg=CARD)
    row.pack(fill="x", pady=2)
    tk.Label(row, text=label, bg=CARD, fg=FG, width=22, anchor="w",
             font=("TkDefaultFont", 10)).pack(side="left")
    ent = tk.Entry(row, textvariable=var, width=width, bg=BG, fg=FG,
                   insertbackground=FG, relief="flat")
    ent.pack(side="left", padx=4, ipady=3)
    return ent


def primary_button(parent, text: str, cmd, color: str = ACCENT) -> tk.Button:
    return tk.Button(parent, text=text, command=cmd, bg=color, fg="white",
                     activebackground=color, relief="flat",
                     font=("TkDefaultFont", 11, "bold"), padx=18, pady=8)


# ════════════════════════════════════════════════════════════════════════
#  Dialog: Simulation starten
# ════════════════════════════════════════════════════════════════════════
class SimDialog(tk.Toplevel):
    def __init__(self, app: "App"):
        super().__init__(app)
        self.app = app
        self.title("Simulation starten")
        self.configure(bg=BG)
        self.geometry("560x640")
        self.minsize(520, 560)

        tk.Label(self, text="Simulation starten", bg=BG, fg=FG,
                 font=("TkDefaultFont", 15, "bold")).pack(anchor="w", padx=16, pady=(14, 2))
        tk.Label(self, text="MuJoCo + Whole-Body-Policy. Alle Optionen in einem Fenster.",
                 bg=BG, fg=MUTED).pack(anchor="w", padx=16, pady=(0, 8))

        # Defaults spiegeln start.sh (Sim-Zweig).
        self.v_rviz = tk.BooleanVar(value=False)
        self.v_hands = tk.BooleanVar(value=False)
        self.v_open_guis = tk.BooleanVar(value=True)
        self.v_nav = tk.BooleanVar(value=False)
        self.v_rebuild = tk.BooleanVar(value=False)
        self.v_env = tk.StringVar(value="Standard — aktuelles Terrain (scene.xml)")
        self.v_rt = tk.StringVar(value="1.0")

        # ── Umgebung ──
        s = section(self, "Umgebung (G1 bleibt gleich, nur die Welt wechselt)")
        self._scene_paths: dict[str, str] = {"Standard — aktuelles Terrain (scene.xml)": ""}
        names = ["Standard — aktuelles Terrain (scene.xml)"]
        for p in list_scenes():
            names.append(p.stem)
            self._scene_paths[p.stem] = p.stem
        self.cb_env = ttk.Combobox(s, textvariable=self.v_env, values=names,
                                   state="readonly", width=44)
        self.cb_env.pack(anchor="w", pady=2)
        tk.Label(s, text="Umgebungen anlegen/bearbeiten: Hauptmenue -> 'Umgebungen bearbeiten'.",
                 bg=CARD, fg=MUTED, font=("TkDefaultFont", 9)).pack(anchor="w", pady=(2, 0))

        # ── Visualisierung / Features ──
        s = section(self, "Visualisierung & Features")
        toggle_row(s, "RViz mitstarten", self.v_rviz,
                   "CoM-/TF-Visualisierung (MuJoCo-Fenster kommt immer)")
        toggle_row(s, "Navigation mitstarten", self.v_nav,
                   "dijkstra_planner + nav2point + Sim-Glue")

        # ── Haende ──
        s = section(self, "Inspire-FTP-Haende")
        toggle_row(s, "Inspire-Haende (Finger steuerbar + GUIs)", self.v_hands,
                   "sonst Rubber-Hand")
        self._open_row = toggle_row(s, "Hand-GUIs automatisch im Browser oeffnen",
                                    self.v_open_guis)
        self.v_hands.trace_add("write", lambda *_: self._sync_hands())
        self._sync_hands()

        # ── Erweitert ──
        s = section(self, "Erweitert")
        field_row(s, "Realtime-Faktor (1.0=Echtzeit)", self.v_rt, width=8)
        toggle_row(s, "Docker-Images vor dem Start neu bauen (--build)", self.v_rebuild,
                   "nach Code-/Dockerfile-Aenderungen")

        # ── Aktion ──
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", side="bottom", padx=16, pady=12)
        primary_button(foot, "▶  Simulation starten", self._start, GREEN).pack(side="left")
        tk.Button(foot, text="Abbrechen", command=self.destroy, bg=CARD, fg=FG,
                  relief="flat", padx=12, pady=8).pack(side="right")

    def _sync_hands(self) -> None:
        state = "normal" if self.v_hands.get() else "disabled"
        for child in self._open_row.winfo_children():
            try:
                child.configure(state=state)
            except tk.TclError:
                pass

    def _start(self) -> None:
        env = os.environ.copy()
        env["G1_MODE"] = "sim"
        env["USE_RVIZ"] = "true" if self.v_rviz.get() else "false"
        env["G1_INSPIRE_HANDS"] = "1" if self.v_hands.get() else "0"
        env["OPEN_GUIS"] = "true" if (self.v_hands.get() and self.v_open_guis.get()) else "false"
        env["G1_ENABLE_NAV"] = "1" if self.v_nav.get() else "0"
        env["G1_ENV"] = self._scene_paths.get(self.v_env.get(), "")
        rt = self.v_rt.get().strip() or "1.0"
        try:
            float(rt)
        except ValueError:
            messagebox.showerror("Ungueltig", "Realtime-Faktor muss eine Zahl sein (z.B. 1.0).",
                                 parent=self)
            return
        env["SIM_REALTIME_FACTOR"] = rt

        argv = ["bash", str(START_SH), "--yes"]
        if self.v_rebuild.get():
            argv.append("--build")

        stop_cmd = ["docker", "compose", "--profile", "sim", "down", "--remove-orphans"]
        self.destroy()
        ProcessConsole(self.app, "Simulation", argv, cwd=HERE, env=env,
                       stop_cmd=stop_cmd,
                       stop_note="Stoppt den Sim-Stack (docker compose down).")


# ════════════════════════════════════════════════════════════════════════
#  Dialog: Echten Roboter starten
# ════════════════════════════════════════════════════════════════════════
class RealDialog(tk.Toplevel):
    def __init__(self, app: "App"):
        super().__init__(app)
        self.app = app
        self.title("Echten Roboter starten")
        self.configure(bg=BG)
        self.geometry("580x760")
        self.minsize(540, 640)

        tk.Label(self, text="Echten Roboter starten", bg=BG, fg=RED,
                 font=("TkDefaultFont", 15, "bold")).pack(anchor="w", padx=16, pady=(14, 2))
        tk.Label(self, text="Unitree-Loco + Arme + Haende ueber LAN. Der Roboter bewegt sich!",
                 bg=BG, fg=MUTED).pack(anchor="w", padx=16, pady=(0, 8))

        # Defaults spiegeln start.sh (Real-Zweig).
        self.v_hands = tk.BooleanVar(value=True)
        self.v_open_guis = tk.BooleanVar(value=True)
        self.v_rviz = tk.BooleanVar(value=True)
        self.v_lidar = tk.BooleanVar(value=False)
        self.v_rebuild = tk.BooleanVar(value=False)
        self.v_left = tk.StringVar(value="192.168.123.210")
        self.v_right = tk.StringVar(value="192.168.123.211")
        self.v_port = tk.StringVar(value="6000")
        self.v_vx = tk.StringVar(value="0.4")
        self.v_vy = tk.StringVar(value="0.3")
        self.v_vyaw = tk.StringVar(value="0.4")
        self.v_joy = tk.StringVar(value="Wireless Controller")
        self.v_confirm = tk.BooleanVar(value=False)

        # ── Netzwerk-Interface ──
        s = section(self, "Netzwerk-Interface zum G1 (Roboter-LAN = 192.168.123.x)")
        nics = detect_nics()
        values = []
        default = ""
        self._iface_map: dict[str, str] = {}
        for name, ip in nics:
            label = f"{name}  ({ip or 'keine IPv4'})"
            values.append(label)
            self._iface_map[label] = name
            if ip.startswith("192.168.123."):
                default = label
        if not values:
            values = [""]
        self.v_iface = tk.StringVar(value=default or values[0])
        self.cb_iface = ttk.Combobox(s, textvariable=self.v_iface, values=values, width=42)
        self.cb_iface.pack(anchor="w", pady=2)
        tk.Label(s, text="Kein passendes Interface? Namen direkt eintippen (z.B. enp3s0).",
                 bg=CARD, fg=MUTED, font=("TkDefaultFont", 9)).pack(anchor="w")

        # ── Haende ──
        s = section(self, "Inspire-FTP-Haende (Modbus TCP im Roboter-LAN)")
        toggle_row(s, "Haende ansteuern (Hand-Bridge + GUIs)", self.v_hands)
        self._hand_box = tk.Frame(s, bg=CARD)
        self._hand_box.pack(fill="x")
        field_row(self._hand_box, "IP linke Hand", self.v_left)
        field_row(self._hand_box, "IP rechte Hand", self.v_right)
        field_row(self._hand_box, "Modbus-Port", self.v_port, width=8)
        toggle_row(self._hand_box, "Hand-GUIs automatisch oeffnen", self.v_open_guis)
        self.v_hands.trace_add("write", lambda *_: self._sync_hands())
        self._sync_hands()

        # ── Steuerung / Visualisierung ──
        s = section(self, "Steuerung & Visualisierung")
        toggle_row(s, "RViz mitstarten (IK-Marker der Arm-Manipulation)", self.v_rviz)
        field_row(s, "Joystick-Name (evdev)", self.v_joy, width=24)
        toggle_row(s, "LiDAR aktivieren (G1_ENABLE_LIDAR)", self.v_lidar,
                   "nur mit Livox-Setup")

        # ── Walk-Limits ──
        s = section(self, "Walk-Limits (konservativ fuer erste Tests)")
        row = tk.Frame(s, bg=CARD)
        row.pack(fill="x", pady=2)
        for lbl, var in (("vx", self.v_vx), ("vy", self.v_vy), ("vyaw", self.v_vyaw)):
            tk.Label(row, text=lbl, bg=CARD, fg=FG).pack(side="left", padx=(0, 2))
            tk.Entry(row, textvariable=var, width=6, bg=BG, fg=FG,
                     insertbackground=FG, relief="flat").pack(side="left", padx=(0, 12), ipady=2)

        s = section(self, "Erweitert")
        toggle_row(s, "Docker-Images vor dem Start neu bauen (--build)", self.v_rebuild)

        # ── SICHERHEITS-GATE ──
        gate = tk.Frame(self, bg="#3a2326", bd=0)
        gate.pack(fill="x", padx=14, pady=8)
        tk.Label(gate, text="⚠  ACHTUNG: ECHTER ROBOTER", bg="#3a2326", fg=RED,
                 font=("TkDefaultFont", 11, "bold")).pack(anchor="w", padx=12, pady=(8, 0))
        tk.Label(gate,
                 text=("Kein Auto-Start: der Roboter bewegt sich erst nach Streamdeck-\n"
                       "Kommandos. E-STOP = Damp = Roboter sackt ZUSAMMEN (sichern!).\n"
                       "Checkliste vorher: g1pilot/REAL_TESTING.md"),
                 bg="#3a2326", fg=FG, justify="left",
                 font=("TkDefaultFont", 9)).pack(anchor="w", padx=12, pady=(2, 6))
        tk.Checkbutton(gate,
                       text="Ich habe die Sicherheitshinweise gelesen und der Bereich ist frei.",
                       variable=self.v_confirm, bg="#3a2326", fg=FG, selectcolor=BG,
                       activebackground="#3a2326", activeforeground=FG,
                       command=self._sync_confirm).pack(anchor="w", padx=12, pady=(0, 8))

        # ── Aktion ──
        foot = tk.Frame(self, bg=BG)
        foot.pack(fill="x", side="bottom", padx=16, pady=12)
        self.start_btn = primary_button(foot, "🤖  Echten Roboter starten", self._start, RED)
        self.start_btn.configure(state="disabled")
        self.start_btn.pack(side="left")
        tk.Button(foot, text="Abbrechen", command=self.destroy, bg=CARD, fg=FG,
                  relief="flat", padx=12, pady=8).pack(side="right")

    def _sync_hands(self) -> None:
        state = "normal" if self.v_hands.get() else "disabled"
        for child in self._hand_box.winfo_children():
            for w in child.winfo_children():
                try:
                    w.configure(state=state)
                except tk.TclError:
                    pass

    def _sync_confirm(self) -> None:
        self.start_btn.configure(state="normal" if self.v_confirm.get() else "disabled")

    def _start(self) -> None:
        if not self.v_confirm.get():
            return
        iface_label = self.v_iface.get().strip()
        iface = self._iface_map.get(iface_label, iface_label.split()[0] if iface_label else "")
        if not iface:
            messagebox.showerror("Interface fehlt",
                                 "Bitte ein Netzwerk-Interface waehlen oder eintippen.",
                                 parent=self)
            return

        env = os.environ.copy()
        env["G1_MODE"] = "real"
        env["G1_REAL_CONFIRM"] = "1"  # Sicherheits-Gate von start.sh im --yes-Modus
        env["ROBOT_INTERFACE"] = iface
        env["G1_INSPIRE_HANDS"] = "1" if self.v_hands.get() else "0"
        env["OPEN_GUIS"] = "true" if (self.v_hands.get() and self.v_open_guis.get()) else "false"
        env["G1_HAND_LEFT_HOST"] = self.v_left.get().strip() or "192.168.123.210"
        env["G1_HAND_RIGHT_HOST"] = self.v_right.get().strip() or "192.168.123.211"
        env["G1_HAND_PORT"] = self.v_port.get().strip() or "6000"
        env["USE_RVIZ"] = "true" if self.v_rviz.get() else "false"
        env["G1_ENABLE_LIDAR"] = "1" if self.v_lidar.get() else "0"
        env["JOYSTICK_NAME"] = self.v_joy.get().strip() or "Wireless Controller"
        env["G1_MAX_VX"] = self.v_vx.get().strip() or "0.4"
        env["G1_MAX_VY"] = self.v_vy.get().strip() or "0.3"
        env["G1_MAX_VYAW"] = self.v_vyaw.get().strip() or "0.4"

        argv = ["bash", str(START_SH), "--yes"]
        if self.v_rebuild.get():
            argv.append("--build")

        stop_cmd = ["docker", "compose", "--profile", "real", "down", "--remove-orphans"]
        self.destroy()
        ProcessConsole(self.app, "Echter Roboter", argv, cwd=HERE, env=env,
                       stop_cmd=stop_cmd,
                       stop_note="Stoppt den Real-Stack (docker compose down).")


# ════════════════════════════════════════════════════════════════════════
#  Dialog: Umgebungen bearbeiten (scene_editor)
# ════════════════════════════════════════════════════════════════════════
class SceneDialog(tk.Toplevel):
    def __init__(self, app: "App"):
        super().__init__(app)
        self.app = app
        self.title("Umgebungen bearbeiten")
        self.configure(bg=BG)
        self.geometry("620x560")
        self.minsize(560, 480)

        tk.Label(self, text="Umgebungen bearbeiten", bg=BG, fg=FG,
                 font=("TkDefaultFont", 15, "bold")).pack(anchor="w", padx=16, pady=(14, 2))
        tk.Label(self, text="Szenen aus scene_editor/scenes/ — dieselben, die beim Sim-Start "
                            "waehlbar sind.", bg=BG, fg=MUTED).pack(anchor="w", padx=16)

        self.v_hands = tk.BooleanVar(value=False)

        if not scene_editor_ready():
            self._show_setup_needed()
            return
        self._build_editor_ui()

    # ── Fall: virtualenv fehlt ────────────────────────────────────────────
    def _show_setup_needed(self) -> None:
        box = section(self, "Einmaliges Setup noetig")
        tk.Label(box, text="Der Scene-Editor braucht ein virtualenv (scene_editor/.venv).\n"
                           "Das wird einmalig eingerichtet (Internet noetig, danach gecacht).",
                 bg=CARD, fg=FG, justify="left").pack(anchor="w", pady=(0, 8))
        primary_button(box, "Setup jetzt ausfuehren", self._run_setup, ACCENT).pack(anchor="w")

    def _run_setup(self) -> None:
        ProcessConsole(self.app, "Scene-Editor Setup", ["bash", str(SETUP_SH)],
                       cwd=SCENE_DIR, env=os.environ.copy(),
                       stop_note="Bricht das Setup ab.")
        tk.Label(self, text="Nach erfolgreichem Setup dieses Fenster schliessen und erneut "
                            "'Umgebungen bearbeiten' oeffnen.", bg=BG, fg=AMBER,
                 wraplength=560, justify="left").pack(padx=16, pady=10)

    # ── Fall: bereit ─────────────────────────────────────────────────────
    def _build_editor_ui(self) -> None:
        s = section(self, "Vorhandene Umgebungen")
        listwrap = tk.Frame(s, bg=CARD)
        listwrap.pack(fill="both", expand=True)
        self.listbox = tk.Listbox(listwrap, height=9, bg=BG, fg=FG,
                                  selectbackground=ACCENT, relief="flat",
                                  activestyle="none", exportselection=False)
        sb = ttk.Scrollbar(listwrap, orient="vertical", command=self.listbox.yview)
        self.listbox.configure(yscrollcommand=sb.set)
        self.listbox.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self._reload_scenes()

        toggle_row(s, "Beim Ansehen 'mit G1' die Inspire-Haende laden", self.v_hands)

        # Aktionen fuer die gewaehlte Szene.
        act = section(self, "Aktion fuer die gewaehlte Umgebung")
        grid = tk.Frame(act, bg=CARD)
        grid.pack(fill="x")
        primary_button(grid, "✎  Im Editor bearbeiten",
                       lambda: self._run_scene("edit"), ACCENT).grid(
            row=0, column=0, padx=4, pady=4, sticky="ew")
        primary_button(grid, "👁  Nur Umgebung ansehen",
                       lambda: self._run_scene("view"), CARD).grid(
            row=0, column=1, padx=4, pady=4, sticky="ew")
        primary_button(grid, "🤖  Mit G1 ansehen",
                       lambda: self._run_scene("with-g1"), GREEN).grid(
            row=1, column=0, padx=4, pady=4, sticky="ew")
        tk.Button(grid, text="↻  Liste aktualisieren", command=self._reload_scenes,
                  bg=CARD, fg=FG, relief="flat", padx=10, pady=8).grid(
            row=1, column=1, padx=4, pady=4, sticky="ew")
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        # Neu anlegen.
        new = section(self, "Neue Umgebung")
        newrow = tk.Frame(new, bg=CARD)
        newrow.pack(fill="x")
        primary_button(newrow, "＋  Leere Umgebung im Editor",
                       lambda: self._run_cmd(["new"], "Neue Umgebung"), ACCENT).pack(
            side="left", padx=(0, 6))
        tk.Button(newrow, text="✨  Aus Text-Prompt generieren", command=self._run_prompt,
                  bg=CARD, fg=FG, relief="flat", padx=12, pady=8).pack(side="left")
        tk.Label(new, text="Der Editor oeffnet einen lokalen Webserver (http://127.0.0.1:8080). "
                           "Export landet automatisch in scenes/.",
                 bg=CARD, fg=MUTED, font=("TkDefaultFont", 9),
                 wraplength=560, justify="left").pack(anchor="w", pady=(6, 0))

    def _reload_scenes(self) -> None:
        self.listbox.delete(0, "end")
        self._scenes = list_scenes()
        if not self._scenes:
            self.listbox.insert("end", "(noch keine Umgebungen — 'Leere Umgebung' anlegen)")
            self.listbox.configure(state="disabled")
            return
        self.listbox.configure(state="normal")
        for p in self._scenes:
            self.listbox.insert("end", p.stem)
        self.listbox.selection_set(0)

    def _selected_scene(self) -> Path | None:
        if not getattr(self, "_scenes", None):
            return None
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showinfo("Keine Auswahl", "Bitte zuerst eine Umgebung waehlen.",
                                parent=self)
            return None
        return self._scenes[sel[0]]

    def _run_scene(self, cmd: str) -> None:
        scene = self._selected_scene()
        if scene is None:
            return
        env = os.environ.copy()
        env["G1_INSPIRE_HANDS"] = "1" if self.v_hands.get() else "0"
        title = {"edit": "Editor", "view": "Viewer", "with-g1": "Viewer + G1"}[cmd]
        self._run_cmd([cmd, str(scene)], f"{title}: {scene.stem}", env=env)

    def _run_prompt(self) -> None:
        text = simpledialog.askstring(
            "Umgebung aus Text",
            "Beschreibe die Umgebung (Englisch funktioniert am besten):",
            parent=self)
        if not text:
            return
        self._run_cmd(["prompt", text], "Umgebung generieren")

    def _run_cmd(self, args: list[str], title: str, env: dict | None = None) -> None:
        ProcessConsole(self.app, f"Scene-Editor — {title}",
                       ["bash", str(LAUNCH_SH), *args],
                       cwd=SCENE_DIR, env=env or os.environ.copy(),
                       stop_note="Beendet den Editor/Viewer-Prozess.")


# ════════════════════════════════════════════════════════════════════════
#  Hauptfenster
# ════════════════════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("G1 — Startmenue")
        self.configure(bg=BG)
        self.geometry("560x580")
        self.minsize(520, 540)

        # Kopf.
        head = tk.Frame(self, bg=BG)
        head.pack(fill="x", padx=24, pady=(22, 6))
        tk.Label(head, text="G1 Robot Control", bg=BG, fg=FG,
                 font=("TkDefaultFont", 20, "bold")).pack(anchor="w")
        tk.Label(head, text="Simulation, echter Roboter und Umgebungen — ohne Terminal.",
                 bg=BG, fg=MUTED, font=("TkDefaultFont", 11)).pack(anchor="w")

        # Grosse Buttons.
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=24, pady=8)
        self._big_button(body, "▶", "Simulation starten",
                         "MuJoCo + Whole-Body-Policy — gefahrlos testen",
                         GREEN, self.open_sim)
        self._big_button(body, "🤖", "Echten Roboter starten",
                         "G1 per LAN — Loco + Arme + Haende (Roboter bewegt sich!)",
                         RED, self.open_real)
        self._big_button(body, "🏗", "Umgebungen bearbeiten",
                         "Szenen anlegen, bearbeiten und mit dem G1 ansehen",
                         ACCENT, self.open_scenes)

        # Sekundaerzeile.
        sec = tk.Frame(self, bg=BG)
        sec.pack(fill="x", padx=24, pady=(4, 8))
        tk.Button(sec, text="■  Laufende Stacks stoppen", command=self.stop_all,
                  bg=CARD, fg=FG, relief="flat", padx=12, pady=6).pack(side="left")
        self.docs_btn = ttk.Menubutton(sec, text="Dokumentation")
        docmenu = tk.Menu(self.docs_btn, tearoff=0)
        for label, path in DOCS:
            docmenu.add_command(label=label, command=lambda p=path: open_path(p))
        self.docs_btn["menu"] = docmenu
        self.docs_btn.pack(side="left", padx=8)
        tk.Button(sec, text="Beenden", command=self.destroy, bg=CARD, fg=FG,
                  relief="flat", padx=12, pady=6).pack(side="right")

        # Statuszeile (Docker/Editor-Bereitschaft).
        self.status = tk.Label(self, text="", bg="#171a20", fg=MUTED, anchor="w",
                               font=("TkFixedFont", 9))
        self.status.pack(fill="x", side="bottom")
        self.after(200, self.refresh_status)

    def _big_button(self, parent, icon, title, subtitle, color, cmd):
        card = tk.Frame(parent, bg=CARD, cursor="hand2")
        card.pack(fill="x", pady=7)
        bar = tk.Frame(card, bg=color, width=6)
        bar.pack(side="left", fill="y")
        inner = tk.Frame(card, bg=CARD)
        inner.pack(side="left", fill="both", expand=True, padx=14, pady=12)
        tk.Label(inner, text=f"{icon}  {title}", bg=CARD, fg=FG,
                 font=("TkDefaultFont", 14, "bold")).pack(anchor="w")
        tk.Label(inner, text=subtitle, bg=CARD, fg=MUTED,
                 font=("TkDefaultFont", 10)).pack(anchor="w")
        # Klick auf die ganze Karte.
        for w in (card, inner, bar, *inner.winfo_children()):
            w.bind("<Button-1>", lambda _e, c=cmd: c())
        return card

    # ── Aktionen ──────────────────────────────────────────────────────────
    def _docker_guard(self) -> bool:
        if docker_ready():
            return True
        return messagebox.askyesno(
            "Docker nicht erreichbar",
            "Docker scheint nicht zu laufen oder ist nicht installiert.\n"
            "Trotzdem fortfahren?")

    def open_sim(self):
        if self._docker_guard():
            SimDialog(self)

    def open_real(self):
        if self._docker_guard():
            RealDialog(self)

    def open_scenes(self):
        SceneDialog(self)

    def stop_all(self):
        if not messagebox.askyesno("Stoppen",
                                   "Sim- UND Real-Stack stoppen (docker compose down)?"):
            return
        # Beide Profile herunterfahren; harmlos, wenn nichts laeuft.
        cmd = ["bash", "-c",
               "docker compose --profile sim down --remove-orphans; "
               "docker compose --profile real down --remove-orphans"]
        ProcessConsole(self, "Stacks stoppen", cmd, cwd=HERE, env=os.environ.copy(),
                       stop_note="")

    def refresh_status(self):
        # Docker-Check kann bis zu einigen Sekunden dauern -> im Thread, damit
        # das Fenster nicht einfriert. Ergebnis via after() zurueck in den UI-Thread.
        editor = "Scene-Editor: bereit" if scene_editor_ready() else "Scene-Editor: Setup noetig"
        scenes = f"Umgebungen: {len(list_scenes())}"
        self.status.configure(text=f"  Docker: pruefe…   |   {editor}   |   {scenes}")

        def worker():
            docker = "Docker: ok" if docker_ready() else "Docker: nicht erreichbar"
            text = f"  {docker}   |   {editor}   |   {scenes}"
            self.after(0, lambda: self.status.configure(text=text))

        threading.Thread(target=worker, daemon=True).start()


def main() -> int:
    app = App()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
