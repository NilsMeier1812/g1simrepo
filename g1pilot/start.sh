#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════
#  start.sh — Interaktives Start-Menue fuer den G1-Sim-Stack.
#
#  Fragt vor dem Start ab, WAS man haben will, setzt die passenden
#  Umgebungsvariablen und startet den Stack. Ersetzt das manuelle Setzen
#  von HOLD_BASE_MODE / SIM_LOCKSTEP / USE_RVIZ / BAL_MODE und das Merken
#  der richtigen run_*.sh-Variante.
#
#  Abgefragt:
#    1) Balance-Regler  -> BAL_MODE   (pd = steht perfekt | policy = RL)
#    2) RViz mitstarten -> USE_RVIZ   (an/aus; dank Lockstep gefahrlos)
#    3) Lockstep        -> SIM_LOCKSTEP (an = deterministische 50 Hz, empfohlen)
#    4) Images neu bauen-> --build
#
#  Alles hat sinnvolle Defaults — einfach ENTER druecken nimmt den Default.
#  Nicht-interaktiv nutzbar via Env, z.B.:
#    BAL_MODE=policy USE_RVIZ=true ./start.sh --yes
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

# ── 1) Balance-Regler ───────────────────────────────────────────────────
ask_menu "1) Balance-Regler?" 1 "${BAL_MODE:-}" \
  "PD   — modellbasiert, steht perfekt am Platz (empfohlen)|pd" \
  "Policy — RL-Policy (eher fuers Laufen)|policy"
export BAL_MODE="$REPLY_VALUE"

# ── 2) RViz ─────────────────────────────────────────────────────────────
ask_menu "2) RViz mitstarten? (MuJoCo-Fenster kommt immer)" 2 "${USE_RVIZ:-}" \
  "Ja  — RViz an (CoM-/TF-Visualisierung; dank Lockstep gefahrlos)|true" \
  "Nein — nur MuJoCo-Fenster|false"
export USE_RVIZ="$REPLY_VALUE"

# ── 3) Lockstep ─────────────────────────────────────────────────────────
ask_menu "3) Lockstep (deterministische 50-Hz-Regelrate)?" 1 "${SIM_LOCKSTEP:-}" \
  "An  — empfohlen, PC-unabhaengig stabil|1" \
  "Aus — alte Wall-Clock-Taktung (nur Debug)|0"
export SIM_LOCKSTEP="$REPLY_VALUE"

# ── 4) Rebuild ──────────────────────────────────────────────────────────
ask_menu "4) Docker-Images vor dem Start neu bauen?" 1 "" \
  "Nein — vorhandene Images nutzen (schnell)|0" \
  "Ja  — --build (nach Code-/Dockerfile-Aenderungen)|1"
if [ "$REPLY_VALUE" = "1" ]; then PASSTHRU+=("--build"); fi

# ── Feste Sim-Voreinstellungen (Loco-Modus, Basis frei) ─────────────────
export HOLD_BASE_MODE=off            # Basis frei -> Balancer regelt die Beine
export SIM_REALTIME_FACTOR=${SIM_REALTIME_FACTOR:-1.0}  # im Lockstep irrelevant
export G1_SIM_MODE=true

# ── Zusammenfassung ─────────────────────────────────────────────────────
echo -e "${B}Starte mit:${R}"
echo -e "   Balance-Regler : ${G}BAL_MODE=${BAL_MODE}${R}"
echo -e "   RViz           : ${G}USE_RVIZ=${USE_RVIZ}${R}"
echo -e "   Lockstep       : ${G}SIM_LOCKSTEP=${SIM_LOCKSTEP}${R}"
echo -e "   Basis          : ${G}HOLD_BASE_MODE=${HOLD_BASE_MODE}${R} (Loco-Modus)"
[ "${#PASSTHRU[@]}" -gt 0 ] && echo -e "   compose-Args   : ${G}${PASSTHRU[*]}${R}"
echo

# ── X11 freigeben (MuJoCo-Viewer + ggf. RViz brauchen den X-Server) ─────
xhost +local:root >/dev/null 2>&1 || \
  echo -e "${Y}[start] WARN: 'xhost' nicht verfuegbar - laeuft hier ein X-Server?${R}"

# ── Reste eines frueheren Laufs sauber entfernen ────────────────────────
docker compose --profile sim down --remove-orphans

# ── Hochfahren ──────────────────────────────────────────────────────────
echo -e "${G}[start] docker compose --profile sim up ${PASSTHRU[*]}${R}"
exec docker compose --profile sim up "${PASSTHRU[@]}"
