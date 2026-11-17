#!/usr/bin/env bash
# interleaved A/B: lerobot vs jitterbug

set -euo pipefail
trap 'echo interrupted; exit 130' INT

HZ=${HZ:-200}
SECS=${SECS:-60}
ROUNDS=${ROUNDS:-5}
PAUSE=${PAUSE:-5}
LEADER=${LEADER:-/dev/ttyACM0}
FOLLOWER=${FOLLOWER:-/dev/ttyACM1}
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-$HERE/results/$(date +%Y%m%d_%H%M%S)}
JB_BIN=${JB_BIN:-$HERE/../target/release/jitterbug}
PY=${PY:-/home/conjeevaram/manipulation/.venv/bin/python}
LEROBOT_EXTRA=${LEROBOT_EXTRA:-}
if [[ -z "$LEROBOT_EXTRA" ]]; then
  echo "error: LEROBOT_EXTRA is empty; pass --robot.id and --teleop.id, e.g." >&2
  echo "  LEROBOT_EXTRA='--robot.id=my_follower --teleop.id=my_leader' $0" >&2
  echo "(a missing calibration id may block on an interactive prompt)" >&2
  exit 1
fi
TICKS=$((HZ * SECS))

mkdir -p "$OUT"
arms=(lerobot jitterbug)

lerobot_once() { # csv
  TELEOP_CSV="$1" "$PY" "$HERE/lerobot_instrumented.py" \
    --robot.type=so101_follower --robot.port="$FOLLOWER" \
    --teleop.type=so101_leader --teleop.port="$LEADER" \
    --fps="$HZ" --teleop_time_s="$SECS" $LEROBOT_EXTRA \
    >/dev/null 2>"${1%.csv}.log"
}

run_arm() {
  local arm=$1 r=$2 csv="$OUT/${1}_r${2}.csv"
  echo "round $r: $arm -> $csv"
  case $arm in
    lerobot) lerobot_once "$csv" || { echo "lerobot failed, retrying once"; sleep 5; lerobot_once "$csv"; } ;;
    jitterbug) "$JB_BIN" "$LEADER" "$FOLLOWER" "$TICKS" "$csv" --hz="$HZ" 2>"${csv%.csv}.log" ;;
  esac
}

bad=0 # consecutive failed runs
for ((r = 0; r < ROUNDS; r++)); do
  for ((i = 0; i < ${#arms[@]}; i++)); do
    arm=${arms[$(((i + r) % ${#arms[@]}))]}
    if run_arm "$arm" "$r"; then
      bad=0
    else
      bad=$((bad + 1))
      echo "FAILED: $arm round $r"
    fi
    if ((bad >= 3)); then
      echo "3 failures in a row, stopping"
      exit 1
    fi
    sleep "$PAUSE"
  done
done
echo "done: $OUT"
