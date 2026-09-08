#!/bin/bash
# franka-rt-tune.sh — root; run at boot (franka-rt-tune.service) and on every enp110s0 "up" (NM dispatcher).
# Writes /run/franka/rt-tune.ok ONLY if every hard step succeeds; ethtool steps are soft (logged) except an ACTIVE EEE.
set -u
FCI_IF=enp110s0; LINK_IF=frlink0; RT_CPUS="2 3"; IRQ_CPU=3; IRQ_PRIO=85; MAX_IDLE_LAT_US=10
OK=/run/franka/rt-tune.ok; SUMMARY=/run/franka/rt-tune.txt
mkdir -p /run/franka /tmp/franka; chmod 1777 /tmp/franka; rm -f "$OK"
log(){ echo "franka-rt-tune: $*"; }
fail(){ log "FAIL: $*"; rm -f "$OK"; exit 1; }
warn(){ log "WARN: $*"; }
# 0. kernel isolation asserts (wrong cmdline => refuse; the servo precheck keys off rt-tune.ok)
iso=$(cat /sys/devices/system/cpu/isolated); [ "$iso" = "2-3" ] || fail "cpu isolated='$iso' (expected 2-3) — booted without the RT cmdline?"
nohz=$(cat /sys/devices/system/cpu/nohz_full); [ "$nohz" = "2-3" ] || fail "nohz_full='$nohz' (expected 2-3)"
grep -qw threadirqs /proc/cmdline || fail "threadirqs missing from /proc/cmdline"
# 1. cpufreq: performance + EPP performance on the RT cpus only (others stay powersave for thermal headroom)
for c in $RT_CPUS; do
  d=/sys/devices/system/cpu/cpu$c/cpufreq
  echo performance > $d/scaling_governor || fail "governor cpu$c"
  echo performance > $d/energy_performance_preference || fail "epp cpu$c"
done
# 2. cpuidle: disable states with exit latency > MAX_IDLE_LAT_US on RT cpus (keeps POLL and C1_ACPI; drops C2_ACPI 127us, C3_ACPI 1048us)
for c in $RT_CPUS; do
  for s in /sys/devices/system/cpu/cpu$c/cpuidle/state*; do
    lat=$(cat $s/latency); name=$(cat $s/name)
    if [ "$lat" -gt "$MAX_IDLE_LAT_US" ]; then echo 1 > $s/disable || fail "disable $name on cpu$c"; log "cpu$c: disabled $name (latency ${lat}us)"
    else echo 0 > $s/disable; fi
  done
done
# 3. robot NIC MSI-X IRQs -> cpu3, their irq threads -> FIFO 85 (vectors are requested when the interface is opened; wait up to 60 s)
irqs=""
for i in $(seq 1 60); do
  irqs=$(awk -v ifn="$FCI_IF" '$NF ~ ("^" ifn "(-[0-9]+)?$") { sub(":", "", $1); print $1 }' /proc/interrupts)
  [ -n "$irqs" ] && break; sleep 1
done
[ -n "$irqs" ] || fail "no IRQs named ${FCI_IF}* in /proc/interrupts after 60 s (r8126 not bound / interface never opened)"
n=0
for i in $irqs; do
  echo $IRQ_CPU > /proc/irq/$i/smp_affinity_list || fail "smp_affinity irq $i"
  grep -qx "$IRQ_CPU" /proc/irq/$i/effective_affinity_list || warn "irq $i effective affinity is $(cat /proc/irq/$i/effective_affinity_list)"
  pid=$(pgrep "^irq/$i-" | head -n1)
  [ -n "$pid" ] || fail "no irq thread for irq $i (threadirqs not active?)"
  chrt -f -p $IRQ_PRIO "$pid" || fail "chrt FIFO $IRQ_PRIO on pid $pid"
  n=$((n+1))
done
log "pinned $n $FCI_IF IRQs to cpu$IRQ_CPU, irq threads FIFO $IRQ_PRIO"
# 4. ethtool (soft): coalescing off, EEE off, pause off, offloads off, single queue
et(){ if out=$(ethtool "$@" 2>&1); then log "ethtool $*: ok"; else warn "ethtool $*: ${out:-unsupported}"; fi; }
et -C $FCI_IF rx-usecs 0 tx-usecs 0 rx-frames 1 tx-frames 1
et --set-eee $FCI_IF eee off
et -A $FCI_IF autoneg off rx off tx off
et -K $FCI_IF gro off gso off tso off lro off
cur=$(ethtool -l $FCI_IF 2>/dev/null | awk '/Current hardware settings/{f=1} f && /Combined:/{print $2; exit}')
if [ "${cur:-1}" != 1 ]; then et -L $FCI_IF combined 1; else log "queues: combined=${cur:-n/a} (no -L needed)"; fi
ethtool --show-eee $FCI_IF 2>/dev/null | grep -q 'EEE status: enabled - active' && fail "EEE is ACTIVE on $FCI_IF (modprobe eee_enable=0 not applied?)"
# 5. no forwarding on the robot NIC / rog link (docker keeps global ip_forward=1; nftables + DOCKER-USER also drop)
sysctl -q -w net.ipv4.conf.$FCI_IF.forwarding=0 || fail "sysctl forwarding $FCI_IF"
if [ -d /sys/class/net/$LINK_IF ]; then sysctl -q -w net.ipv4.conf.$LINK_IF.forwarding=0 || fail "sysctl forwarding $LINK_IF"; else log "$LINK_IF absent (dongle not plugged) — skipping its sysctl"; fi
# 6. summary + ok marker
{
  echo "ok=$(date -Is) isolated=$iso nohz_full=$nohz irqs=$(echo $irqs | tr ' ' ',')->cpu$IRQ_CPU/FIFO$IRQ_PRIO"
  for c in $RT_CPUS; do echo "cpu$c gov=$(cat /sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor) epp=$(cat /sys/devices/system/cpu/cpu$c/cpufreq/energy_performance_preference) idle_disable=$(cat /sys/devices/system/cpu/cpu$c/cpuidle/state*/disable | tr '\n' ' ')"; done
} > "$SUMMARY"
touch "$OK"; log "OK: $(head -n1 $SUMMARY)"
