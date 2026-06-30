#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════
#  start.sh — Interaktives Start-Menue fuer den G1-Sim-Stack.
#
#  Fragt vor dem Start ab, WAS man haben will, setzt die passenden
#  Umgebungsvariablen und startet den Stack. Ersetzt das manuelle Setzen
#  von HOLD_BASE_MODE / SIM_LOCKSTEP / USE_RVIZ und das Merken der richtigen
#  run_*.sh-Variante.
#
#  Abgefragt werden nur ECHTE Start-Entscheidungen (Launch-Zeit). Stehen vs.
#  Laufen ist KEINE davon: die velocity-konditionierte Whole-Body-Policy macht
#  beides (cmd=0 -> stehen, cmd!=0 -> laufen), zur Laufzeit per Streamdeck.
#
#  Abgefragt:
#    1) RViz mitstarten -> USE_RVIZ   (an/aus; dank Lockstep gefahrlos)
#    2) Images neu bauen-> --build
#
#  Lockstep (deterministische 50-Hz-Regelrate, auf Echtzeit gedeckelt) ist im
#  Betrieb IMMER an und daher nicht mehr abgefragt.
#
#  Alles hat sinnvolle Defaults — einfach ENTER druecken nimmt den Default.
#  Nicht-interaktiv nutzbar via Env, z.B.:
#    USE_RVIZ=true ./start.sh --yes
# ════════════════════════════════════════════════════════════════════════
set -e
cd "$(dirname "$0")"

# ── Farben (nur wenn Terminal) ──────────────────────────────────────────
if [ -t 1 ]; then
  B="\033[1m"; DIM="\033[2m"; G="\033[32m"; Y="\033[33m"; C="\033[36m"; R="\033[0m"
else
  B=""; DIM=""; G=""; Y=""; C=""; R=""
fi

ASSUME_YES=0
PASSTHRU=()
for a in "$@"; do
  case "$a" in
    -y|--yes) ASSUME_YES=1 ;;        # keine Rueckfragen, Defaults/Env nehmen
    *) PASSTHRU+=("$a") ;;           # Rest an `docker compose up` (z.B. --build)
  esac
done

# ── Helfer: Einzelauswahl-Menue. $1=Frage, $2=Default-Index(1-basiert),
#    $3=aktueller Env-Wert (hat im --yes-Modus Vorrang; "" = keiner),
#    danach Paare "Label|WERT". Setzt globale Variable REPLY_VALUE. ───────
ask_menu() {
  local prompt="$1"; shift
  local def="$1"; shift
  local current="$1"; shift
  local labels=() values=()
  while [ "$#" -ge 1 ]; do labels+=("${1%%|*}"); values+=("${1##*|}"); shift; done
  # Nicht-interaktiv: vorgegebener Env-Wert gewinnt, sonst Default-Index.
  if [ "$ASSUME_YES" = "1" ]; then
    if [ -n "$current" ]; then REPLY_VALUE="$current"; else REPLY_VALUE="${values[$((def-1))]}"; fi
    return
  fi
  echo -e "${B}${prompt}${R}"
  local i
  for i in "${!labels[@]}"; do
    local n=$((i+1)); local mark="  "
    [ "$n" = "$def" ] && mark="${G}*${R} "
    echo -e "   ${mark}${C}${n})${R} ${labels[$i]}"
  done
  local choice
  read -rp "$(echo -e "   ${DIM}Auswahl [${def}]:${R} ")" choice
  choice="${choice:-$def}"
  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#values[@]}" ]; then
    echo -e "   ${Y}Ungueltig -> Default ${def}.${R}"; choice="$def"
  fi
  REPLY_VALUE="${values[$((choice-1))]}"
  echo
}

clear 2>/dev/null || true
echo -e "${B}╔══════════════════════════════════════════════╗${R}"
echo -e "${B}║      G1 Simulation — Start-Menue              ║${R}"
echo -e "${B}╚══════════════════════════════════════════════╝${R}"
echo -e "${DIM}ENTER = markierter Default (*).${R}\n"

# ── 1) RViz ─────────────────────────────────────────────────────────────
ask_menu "1) RViz mitstarten? (MuJoCo-Fenster kommt immer)" 2 "${USE_RVIZ:-}" \
  "Ja  — RViz an (CoM-/TF-Visualisierung; dank Lockstep gefahrlos)|true" \
  "Nein — nur MuJoCo-Fenster|false"
export USE_RVIZ="$REPLY_VALUE"

# ── 2) Inspire-FTP-Haende ────────────────────────────────────────────────
#   Master-Schalter: waehlt im MuJoCo das Inspire-Finger-Modell UND startet die
#   Hand-Bridge (HTML-GUIs :8766/:8765 + DDS, Finger steuerbar/gemessen in RViz).
ask_menu "2) Inspire-FTP-Haende? (Finger steuerbar + GUIs; sonst Rubber-Hand)" 2 "${G1_INSPIRE_HANDS:-}" \
  "Ja  — Inspire-Modell + Hand-Bridge (GUIs :8766/:8765, echte Kraefte)|1" \
  "Nein — bisheriges Rubber-Hand-Modell|0"
export G1_INSPIRE_HANDS="$REPLY_VALUE"

# ── 2b) Hand-GUIs automatisch im Browser oeffnen (nur mit Inspire-Haenden) ─
if [ "$G1_INSPIRE_HANDS" = "1" ]; then
  ask_menu "2b) Hand-GUIs automatisch im Browser oeffnen (und verbinden)?" 1 "${OPEN_GUIS:-}" \
    "Ja  — Controller + Viewer oeffnen sich selbst und verbinden|true" \
    "Nein — GUIs manuell oeffnen (web/*.html)|false"
  export OPEN_GUIS="$REPLY_VALUE"
else
  export OPEN_GUIS=false
fi

# ── 3) Rebuild ──────────────────────────────────────────────────────────
ask_menu "3) Docker-Images vor dem Start neu bauen?" 1 "" \
  "Nein — vorhandene Images nutzen (schnell)|0" \
  "Ja  — --build (nach Code-/Dockerfile-Aenderungen)|1"
if [ "$REPLY_VALUE" = "1" ]; then PASSTHRU+=("--build"); fi

# ── Feste Sim-Voreinstellungen (Loco-Modus, Basis frei) ─────────────────
export HOLD_BASE_MODE=off            # Basis frei -> die Policy regelt den Koerper
# Lockstep ist im Betrieb IMMER an (deterministische 50-Hz-Regelrate, PC-unabhaengig).
export SIM_LOCKSTEP=1
# Eine velocity-konditionierte Whole-Body-Policy macht Stehen UND Laufen: cmd=0
# -> stehen, cmd!=0 -> laufen. Kein separater Balance-Regler mehr zu waehlen.
# Lockstep ist auf Echtzeit gedeckelt; SIM_REALTIME_FACTOR steuert das Tempo (1.0=Echtzeit).
export SIM_REALTIME_FACTOR=${SIM_REALTIME_FACTOR:-1.0}
export G1_SIM_MODE=true

# ── Zusammenfassung ─────────────────────────────────────────────────────
echo -e "${B}Starte mit:${R}"
echo -e "   RViz           : ${G}USE_RVIZ=${USE_RVIZ}${R}"
_hands_lbl=$( [ "${G1_INSPIRE_HANDS}" = "1" ] && echo "Inspire-FTP (Finger + GUIs)" || echo "Rubber-Hand" )
echo -e "   Haende         : ${G}G1_INSPIRE_HANDS=${G1_INSPIRE_HANDS}${R} ${DIM}(${_hands_lbl})${R}"
[ "${G1_INSPIRE_HANDS}" = "1" ] && echo -e "   Hand-GUIs      : ${G}OPEN_GUIS=${OPEN_GUIS}${R} ${DIM}(Browser-Auto-Open :8766/:8765)${R}"
echo -e "   Lockstep       : ${G}SIM_LOCKSTEP=${SIM_LOCKSTEP}${R} ${DIM}(immer an, auf Echtzeit gedeckelt)${R}"
echo -e "   Basis          : ${G}HOLD_BASE_MODE=${HOLD_BASE_MODE}${R} (Loco-Modus)"
echo -e "   Loco-Policy    : ${G}g1_wholebody${R} ${DIM}(Stehen=cmd0; Laufen via Streamdeck)${R}"
[ "${#PASSTHRU[@]}" -gt 0 ] && echo -e "   compose-Args   : ${G}${PASSTHRU[*]}${R}"
echo

# ── X11 freigeben (MuJoCo-Viewer + ggf. RViz brauchen den X-Server) ─────
xhost +local:root >/dev/null 2>&1 || \
  echo -e "${Y}[start] WARN: 'xhost' nicht verfuegbar - laeuft hier ein X-Server?${R}"

# ── Hand-GUIs nach dem Hochfahren automatisch oeffnen ───────────────────
#  start.sh laeuft auf dem HOST (nicht im Container), kann also direkt einen
#  Browser oeffnen. Wir warten im Hintergrund, bis die Controller-Bridge auf
#  :8766 lauscht (colcon-Build im Container kann dauern), und oeffnen dann beide
#  GUIs mit ?autoconnect=1, sodass sie sich von selbst verbinden.
open_guis_when_ready() {
  local web="$PWD/g1pilot/manipulation/inspire_ftp/web"
  local ctrl="file://$web/hand_controller_viewer.html?autoconnect=1"
  local view="file://$web/inspire_hand_viewer.html?autoconnect=1"

  # WSL2-Erkennung (Kernel-Release enthaelt "microsoft")
  local is_wsl=0
  grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null && is_wsl=1

  # Oeffnet eine URL im System-Browser; gibt 0 zurueck wenn erfolgreich.
  _open_url() {
    local url="$1"
    if [ "$is_wsl" = "1" ]; then
      # WSL2: wslview (aus dem wslu-Paket) ist die sauberste Loesung
      if command -v wslview >/dev/null 2>&1; then
        wslview "$url" >/dev/null 2>&1; return 0
      fi
      # Fallback: Linux-Pfad in Windows-Datei-URL umwandeln, per cmd.exe oeffnen
      if command -v wslpath >/dev/null 2>&1 && command -v cmd.exe >/dev/null 2>&1; then
        local fpath="${url#file://}"
        local query="${fpath#*\?}"; [ "$query" = "$fpath" ] && query=""
        fpath="${fpath%%\?*}"
        local winraw; winraw=$(wslpath -w "$fpath" 2>/dev/null) || return 1
        local winurl="file:///${winraw//\\//}"  # Backslash -> Slash fuer file-URL
        [ -n "$query" ] && winurl="${winurl}?${query}"
        cmd.exe /c start "" "$winurl" >/dev/null 2>&1; return 0
      fi
      command -v explorer.exe >/dev/null 2>&1 && { explorer.exe "$url" >/dev/null 2>&1; return 0; }
    else
      local c
      for c in xdg-open sensible-browser x-www-browser \
               microsoft-edge microsoft-edge-stable msedge \
               firefox google-chrome chromium chromium-browser; do
        command -v "$c" >/dev/null 2>&1 && { "$c" "$url" >/dev/null 2>&1; return 0; }
      done
    fi
    return 1
  }

  # Warten bis die Bridge lauscht (max ~5 min, deckt auch den ersten Build ab).
  local i=0
  while [ $i -lt 600 ]; do
    if (exec 3<>"/dev/tcp/127.0.0.1/8766") 2>/dev/null; then break; fi
    sleep 0.5; i=$((i+1))
  done

  if ! _open_url "$ctrl"; then
    echo -e "${Y}[start] Kein Browser-Opener gefunden - GUIs bitte manuell oeffnen:${R}"
    echo -e "   ${C}$ctrl${R}"
    echo -e "   ${C}$view${R}"
    return
  fi
  sleep 1
  _open_url "$view" >/dev/null 2>&1 || true
}

if [ "$OPEN_GUIS" = "true" ]; then
  ( open_guis_when_ready ) &
fi

# ── Reste eines frueheren Laufs sauber entfernen ────────────────────────
docker compose --profile sim down --remove-orphans

# ── Hochfahren ──────────────────────────────────────────────────────────
echo -e "${G}[start] docker compose --profile sim up ${PASSTHRU[*]}${R}"
exec docker compose --profile sim up "${PASSTHRU[@]}"
