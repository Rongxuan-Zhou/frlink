#!/bin/bash
# verify-rt-host.sh — post-reboot verification of A3 (run as the normal user from a NEW login; sudo only for the preempt debug file)
set -u; f=0
ok(){ echo "PASS $1"; }; bad(){ echo "FAIL $1"; f=$((f+1)); }
grep -qw 'isolcpus=domain,managed_irq,2,3' /proc/cmdline && ok cmdline.isolcpus || bad "cmdline.isolcpus: $(cat /proc/cmdline)"
grep -qw 'preempt=full' /proc/cmdline && ok cmdline.preempt || bad cmdline.preempt
grep -qw 'threadirqs' /proc/cmdline && ok cmdline.threadirqs || bad cmdline.threadirqs
[ "$(cat /sys/devices/system/cpu/isolated)" = 2-3 ] && ok isolated=2-3 || bad "isolated=$(cat /sys/devices/system/cpu/isolated)"
[ "$(cat /sys/devices/system/cpu/nohz_full)" = 2-3 ] && ok nohz_full=2-3 || bad "nohz_full=$(cat /sys/devices/system/cpu/nohz_full)"
[ "$(uname -r)" = 6.8.0-124-generic ] && ok kernel.6.8.0-124 || bad "kernel $(uname -r)"
[ "$(ulimit -Hr)" = 99 ] && ok ulimit.rtprio=99 || bad "ulimit -Hr=$(ulimit -Hr) (new login/reboot needed)"
[ "$(ulimit -Hl)" = unlimited ] && ok ulimit.memlock=unlimited || bad "ulimit -Hl=$(ulimit -Hl)"
id -nG | grep -qw realtime && ok group.realtime || bad group.realtime
id -nG | grep -qw docker && ok group.docker || bad group.docker
[ "$(apt-mark showhold | wc -l)" -ge 9 ] && ok "apt.holds=$(apt-mark showhold | wc -l)" || bad apt.holds
systemctl is-active --quiet irqbalance && bad irqbalance.active || ok irqbalance.inactive
[ "$(systemctl is-enabled sleep.target suspend.target hibernate.target hybrid-sleep.target 2>/dev/null | sort -u)" = masked ] && ok sleep.masked || bad sleep.masked
[ -f /etc/systemd/logind.conf.d/franka.conf ] && ok logind.dropin || bad logind.dropin
[ "$(gsettings get org.gnome.settings-daemon.plugins.power lid-close-ac-action 2>/dev/null)" = "'nothing'" ] && ok gsettings.lid || bad "gsettings.lid (run from a GUI session)"
[ "$(basename "$(readlink /sys/class/net/enp110s0/device/driver)")" = r8126 ] && ok r8126.bound || bad "r8126 not bound to enp110s0"
dkms status 2>/dev/null | grep -qE "^r8126/.*$(uname -r).*installed" && ok dkms.r8126 || bad dkms.r8126
dkms status 2>/dev/null | grep -qE "^nvidia/.*$(uname -r).*installed" && ok dkms.nvidia || bad dkms.nvidia
modprobe -c | grep -q '^options r8126 aspm=0 eee_enable=0' && ok modprobe.r8126 || bad modprobe.r8126
ethtool --show-eee enp110s0 2>/dev/null | grep -q 'EEE status: disabled' && ok eee.disabled || echo "INFO eee: $(ethtool --show-eee enp110s0 2>&1 | grep -m1 'EEE status' || echo 'run with sudo / link down')"
command -v ethtool >/dev/null && ok ethtool || bad ethtool
command -v cyclictest >/dev/null && ok rt-tests || bad rt-tests
[ -d /run/franka ] && [ -d /tmp/franka ] && [ -f /run/lock/franka-robot.lock ] && ok tmpfiles || bad tmpfiles
apt-config dump | grep -q 'Package-Blacklist:: "linux-"' && ok unattended.blacklist || bad unattended.blacklist
nvidia-smi -L >/dev/null 2>&1 && ok nvidia.driver || bad "nvidia-smi failed"
if sudo -n true 2>/dev/null; then p=$(sudo cat /sys/kernel/debug/sched/preempt); case "$p" in *"(full)"*) ok "preempt: $p";; *) bad "preempt: $p";; esac
else echo "INFO run: sudo cat /sys/kernel/debug/sched/preempt   (expect: none voluntary (full))"; fi
echo; if [ $f = 0 ]; then echo "ALL PASS"; else echo "$f FAIL"; exit 1; fi
