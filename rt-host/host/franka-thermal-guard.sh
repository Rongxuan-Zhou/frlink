#!/bin/bash
# franka-thermal-guard.sh [--print] — hottest coretemp sensor in whole °C.
#   --print : just print the value (used by franka-servo-precheck)
#   default : write /run/franka/temp_c (atomic), warn in journal >= 90C, info heartbeat once a minute
set -u
max=0
for h in /sys/class/hwmon/hwmon*; do
  [ "$(cat "$h/name" 2>/dev/null)" = coretemp ] || continue
  for t in "$h"/temp*_input; do v=$(cat "$t" 2>/dev/null || echo 0); [ "$v" -gt "$max" ] && max=$v; done
done
c=$((max/1000))
if [ "${1:-}" = --print ]; then echo "$c"; exit 0; fi
mkdir -p /run/franka
echo "$c" > /run/franka/temp_c.tmp && mv -f /run/franka/temp_c.tmp /run/franka/temp_c
if [ "$c" -ge 90 ]; then logger -t franka-thermal -p daemon.warning "coretemp ${c}C >= 90C (servo start limit 85C; hwlat stalls of 0.3-0.7 ms were measured near 100C)"; fi
if [ "$(date +%S)" = 00 ]; then logger -t franka-thermal -p daemon.info "coretemp ${c}C"; fi
exit 0
