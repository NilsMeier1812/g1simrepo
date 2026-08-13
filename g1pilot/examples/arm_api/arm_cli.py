#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arm_cli.py — Kommandozeilen-Client fuer die Live-Pose-Schnittstelle.
Anleitung: g1pilot/g1pilot/docs/20_arm_api_anleitung.md   ·   Referenz: g1pilot/docs/21_arm_api_technik.md

NUR STANDARD-BIBLIOTHEK (urllib) -- absichtlich: dieses Skript soll auf einem
fremden Rechner ohne ROS und ohne `pip install` sofort laufen. Es ist zugleich
Werkzeug (schnell etwas ausprobieren) und Vorlage (kopieren und in das eigene
Projekt einbauen -- die ganze Logik steckt in `request()`, ~15 Zeilen).

Beispiele:
    python3 arm_cli.py state
    python3 arm_cli.py joints --right 0.3 -0.2 0.0 0.5 0.0 0.0 0.0 --wait 60
    python3 arm_cli.py pose --right 0.35 -0.20 0.10 --rpy 0 0 0 --wait 60
    python3 arm_cli.py pose --left 0.30 0.20 0.10 --quat 0 0 0 1
    python3 arm_cli.py cancel
    python3 arm_cli.py status 3f9a1c4e77b2
    python3 arm_cli.py save "Regal oben" --category Greifen --components arms hands
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request

DEFAULT_URL = "http://localhost:8770"


def request(base_url: str, method: str, path: str, body: dict = None,
            token: str = "", timeout: float = 310.0):
    """Ein Request gegen die Arm-API. -> (HTTP-Code, Antwort als dict).

    Genau das braucht ein fremdes Projekt -- mehr ist die Anbindung nicht.
    WICHTIG: Fehlercodes (400/409/422) tragen die Begruendung im Body, deshalb
    wird HTTPError NICHT als Ausnahme durchgelassen, sondern ausgelesen."""
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(base_url.rstrip("/") + path, data=data, method=method)
    req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("X-Auth-Token", token)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8") or "{}")
    except urllib.error.HTTPError as e:          # 4xx/5xx: Body enthaelt den Grund
        raw = e.read().decode("utf-8", errors="replace")
        try:
            return e.code, json.loads(raw or "{}")
        except ValueError:
            return e.code, {"error": raw.strip() or e.reason}
    except urllib.error.URLError as e:
        raise SystemExit(f"Keine Verbindung zu {base_url}: {e.reason}\n"
                         f"Laeuft der arm_api-Node? (GET {base_url}/arm/health)")


def _print(code: int, payload: dict) -> int:
    """Antwort ausgeben und einen sinnvollen Exit-Code liefern (0 = Ziel
    erreicht bzw. Abfrage ok), damit das Skript in Shell-Pipelines taugt."""
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    state = payload.get("state")
    if code == 200 and state in (None, "reached", "saved"):
        return 0
    if code == 202:
        print("# laeuft noch -- mit --wait <sekunden> auf das Ende warten",
              file=sys.stderr)
        return 0
    hint = payload.get("reason") or payload.get("error") or ""
    print(f"# HTTP {code} {state or ''} {hint}".rstrip(), file=sys.stderr)
    return 1


def cmd_state(args) -> int:
    return _print(*request(args.url, "GET", "/arm/state", token=args.token,
                           timeout=10))


def cmd_health(args) -> int:
    return _print(*request(args.url, "GET", "/arm/health", timeout=10))


def cmd_status(args) -> int:
    path = f"/arm/status/{args.id}" if args.id else "/arm/status"
    return _print(*request(args.url, "GET", path, token=args.token, timeout=10))


def cmd_cancel(args) -> int:
    return _print(*request(args.url, "POST", "/arm/cancel", body={},
                           token=args.token, timeout=10))


def cmd_joints(args) -> int:
    body = {"wait": args.wait}
    if args.right:
        body["right"] = args.right
    if args.left:
        body["left"] = args.left
    if not (args.right or args.left):
        raise SystemExit("Mindestens --right oder --left angeben (je 7 Werte in rad).")
    if args.id:
        body["id"] = args.id
    return _print(*request(args.url, "POST", "/arm/joints", body=body,
                           token=args.token))


def cmd_save(args) -> int:
    """Aktuelle Stellung in die Pose-Datei schreiben (bewegt nichts)."""
    body = {"name": args.name, "category": args.category}
    if args.components:
        body["components"] = args.components
    return _print(*request(args.url, "POST", "/arm/save", body=body,
                           token=args.token, timeout=15))


def cmd_pose(args) -> int:
    if bool(args.rpy) == bool(args.quat):
        raise SystemExit("Genau eines von --rpy (Grad) oder --quat (x y z w) angeben.")
    orient = {"rpy_deg": args.rpy} if args.rpy else {"orientation": args.quat}
    body = {"wait": args.wait}
    for side, xyz in (("right", args.right), ("left", args.left)):
        if xyz:
            body[side] = dict(orient, position=xyz, frame=args.frame)
    if "right" not in body and "left" not in body:
        raise SystemExit("Mindestens --right oder --left angeben (x y z in Metern).")
    if args.id:
        body["id"] = args.id
    if args.no_ee_offset:
        body["apply_ee_offset"] = False
    return _print(*request(args.url, "POST", "/arm/pose", body=body,
                           token=args.token))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Zielposen an g1pilot schicken (siehe g1pilot/docs/20_arm_api_anleitung.md).")
    ap.add_argument("--url", default=DEFAULT_URL, help=f"Default {DEFAULT_URL}")
    ap.add_argument("--token", default="", help="X-Auth-Token, falls konfiguriert")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("health", help="Laeuft der Node?").set_defaults(fn=cmd_health)
    sub.add_parser("state", help="Ist-Gelenkwinkel + letzter Status").set_defaults(
        fn=cmd_state)
    p = sub.add_parser("status", help="Status eines Vorgangs")
    p.add_argument("id", nargs="?", default="", help="Vorgangs-ID (leer = letzter)")
    p.set_defaults(fn=cmd_status)
    sub.add_parser("cancel", help="Laufende Bewegung abbrechen").set_defaults(
        fn=cmd_cancel)

    p = sub.add_parser("joints", help="Gelenkwinkel anfahren (7 Werte je Arm, rad)")
    p.add_argument("--right", type=float, nargs=7, metavar="Q")
    p.add_argument("--left", type=float, nargs=7, metavar="Q")
    p.add_argument("--wait", type=float, default=0.0,
                   help="Sekunden auf den Endzustand warten (0 = nicht warten)")
    p.add_argument("--id", default="", help="eigene Vorgangs-ID")
    p.set_defaults(fn=cmd_joints)

    p = sub.add_parser("save", help="Aktuelle Stellung in die Pose-Datei speichern")
    p.add_argument("name", help="Name der Pose (Schluessel in der Datei)")
    p.add_argument("--category", default="",
                   help="Kategorie/\"Ordner\" (leer = Default-Kategorie)")
    p.add_argument("--components", nargs="+", default=[],
                   metavar="C", help="z.B. arms hands / left_arm right_hand / all "
                                     "(Default: beide Arme)")
    p.set_defaults(fn=cmd_save)

    p = sub.add_parser("pose", help="Kartesische Hand-Pose anfahren (IK + Planung)")
    p.add_argument("--right", type=float, nargs=3, metavar=("X", "Y", "Z"))
    p.add_argument("--left", type=float, nargs=3, metavar=("X", "Y", "Z"))
    p.add_argument("--rpy", type=float, nargs=3, metavar=("R", "P", "Y"),
                   help="Orientierung in Grad")
    p.add_argument("--quat", type=float, nargs=4, metavar=("X", "Y", "Z", "W"))
    p.add_argument("--frame", default="", help="TF-Frame des Ziels (leer = pelvis)")
    p.add_argument("--no-ee-offset", action="store_true",
                   help="Ziel ist die Pose des IK-Frames, nicht die Marker-Pose")
    p.add_argument("--wait", type=float, default=0.0)
    p.add_argument("--id", default="")
    p.set_defaults(fn=cmd_pose)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
