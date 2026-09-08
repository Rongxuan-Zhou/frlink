#!/bin/bash
# franka-thermal-guard.sh [--print|--median] -- package temperature sampling for the RT host.
#   --print  : instantaneous hottest coretemp sensor in whole C. Spiky: a single-core turbo burst from a
#              desktop app pushes the package sensor to 95-100C for < 1 s while the cores sit at 65-70C.
#   --median : median of the last 10 samples written by the 1 Hz timer (/run/franka/temp_hist). This is the
#              value every start gate uses (servo precheck, preflight, status). Falls back to sampling
#              5x over 2 s when the history is short (right after boot).
#   default  : timer body -- write /run/franka/temp_c (instantaneous), append to temp_hist (last 30),
#              write /run/franka/temp_med (median of the last 10), warn in the journal at >= 90C.
set -u
HIST=/run/franka/temp_hist; MED=/run/franka/temp_med; N_MED=10; N_KEEP=30
sample() {
  local max=0 h t v
  for h in /sys/class/hwmon/hwmon*; do
    [ "$(cat "$h/name" 2>/dev/null)" = coretemp ] || continue
    for t in "$h"/temp*_input; do v=$(cat "$t" 2>/dev/null || echo 0); [ "$v" -gt "$max" ] && max=$v; done
  done
  echo $((max/1000))
}
median_of() { sort -n | awk '{a[NR]=$1} END{if(NR==0){print -1; exit} print a[int((NR+1)/2)]}'; }
case "${1:-}" in
  --print)  sample; exit 0 ;;
  --median)
    n=$(wc -l < "$HIST" 2>/dev/null || echo 0)
    if [ "${n:-0}" -ge 5 ]; then tail -n $N_MED "$HIST" | median_of
    else for i in 1 2 3 4 5; do sample; sleep 0.4; done | median_of; fi
    exit 0 ;;
esac
c=$(sample)
mkdir -p /run/franka
echo "$c" > /run/franka/temp_c.tmp && mv -f /run/franka/temp_c.tmp /run/franka/temp_c
{ tail -n $((N_KEEP-1)) "$HIST" 2>/dev/null; echo "$c"; } > "$HIST.tmp" && mv -f "$HIST.tmp" "$HIST"
tail -n $N_MED "$HIST" | median_of > "$MED.tmp" && mv -f "$MED.tmp" "$MED"
if [ "$c" -ge 90 ]; then logger -t franka-thermal -p daemon.warning "coretemp ${c}C >= 90C instantaneous (median $(cat "$MED")C; the servo start gate uses the median, limit 85C)"; fi
if [ "$(date +%S)" = 00 ]; then logger -t franka-thermal -p daemon.info "coretemp ${c}C (median $(cat "$MED")C)"; fi
exit 0
