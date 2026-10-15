#!/usr/bin/env bash
# record_stepfield_run.sh — start the robot stack with a FULLY RECORDED session, so a
# stepfield attempt can be analysed afterwards instead of eyeballed.
#
# It is RL_start_remote_connection.sh with recording switched on and labelled:
#   bag/      rosbag2: /lowstate (joints, IMU, foot forces), /lowcmd, /sportmodestate,
#             /go2/height_scan + /go2/local_heightmap (what the policy was fed),
#             /go2/mount_monitor (live mount-vs-camera.yaml fit), depth image + intrinsics
#             (so the scan can be rebuilt offline with a corrected mount), TF, joystick,
#             mode/estop markers, /rosout.
#   flight/   the policy node's own per-step log: the exact obs vector, raw + commanded
#             actions, scan + scan age, events (needs the policy to engage).
#   camera.yaml + session_metadata.yaml   the mount and labels this run was made with.
#
# Usage (on the Jetson, from the submodule root):
#   ./record_stepfield_run.sh --name field15_a --block 0.15 --terrain "0/15 stepfield, 6 blocks" \
#        [--notes "..."] [--dry-run] [--no-depth] [-- <extra RL_start args>]
#
# Then drive as usual from the web app. Ctrl+C stops everything and finalises the bag.
# Analyse on the PC (copy sessions/<UTC>_<name>/ over first):
#   python3 src/go2_remote_controller/tools/analyze_stepfield_session.py sessions/<UTC>_<name>
#   python3 src/go2_remote_controller/tools/eval_rl_flight.py sessions/<UTC>_<name>/flight/<run>
#
# Disk: depth adds ~14 MB/s on the Jetson; a 3 min attempt is ~2.5 GB. --no-depth keeps
# only the scan (a few MB), which is enough for everything except rebuilding the scan.
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

NAME="stepfield"
BLOCK=""
TERRAIN=""
NOTES=""
DEPTH=1
DRY=0

usage() { grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --name)     NAME="$2";    shift 2 ;;
    --block)    BLOCK="$2";   shift 2 ;;
    --terrain)  TERRAIN="$2"; shift 2 ;;
    --notes)    NOTES="$2";   shift 2 ;;
    --no-depth) DEPTH=0;      shift ;;
    --dry-run)  DRY=1;        shift ;;
    --)         shift; break ;;
    -h|--help)  usage 0 ;;
    *) echo "[record-run] unknown arg: $1" >&2; usage 1 ;;
  esac
done

[ -n "$BLOCK" ] && TERRAIN="${TERRAIN:+$TERRAIN; }block height ${BLOCK} m"

# Free disk check: refuse to start a depth recording into a nearly full card.
FREE_MB="$(df -Pm "$WS_DIR" | awk 'NR==2 {print $4}')"
if [ "$DEPTH" = "1" ] && [ "${FREE_MB:-0}" -lt 4000 ]; then
  echo "[record-run] ⚠️  only ${FREE_MB} MB free; depth recording needs ~1 GB/min. Use --no-depth or free space." >&2
  exit 1
fi

export GO2_RECORD=1
export GO2_RECORD_DEPTH="$DEPTH"
export GO2_RECORD_NAME="$NAME"
export GO2_RECORD_TERRAIN="$TERRAIN"
export GO2_RECORD_NOTES="$NOTES"
[ "$DRY" = "1" ] && export GO2_RL_DRY_RUN=1

echo "[record-run] session name '$NAME'  terrain '$TERRAIN'  depth=$DEPTH  dry_run=$DRY  (${FREE_MB} MB free)"
echo "[record-run] handing over to RL_start_remote_connection.sh -- Ctrl+C ends the run and the recording"
exec "$WS_DIR/RL_start_remote_connection.sh" "$@"
