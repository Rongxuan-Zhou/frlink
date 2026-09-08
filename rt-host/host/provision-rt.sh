#!/bin/bash
# provision-rt.sh — one-shot, idempotent host provisioning for the FR3 realtime host (alienware, Ubuntu 22.04, 6.8.0-124).
# Run: sudo /home/rongxuan_zhou/franka/host/provision-rt.sh   (then reboot)
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
TARGET_USER="${SUDO_USER:-rongxuan_zhou}"
KVER=6.8.0-124-generic
UUID=e2c281cf-83f9-4ec8-97bf-7a224f3af4ae
SRC=/home/rongxuan_zhou/franka/host/etc
CMDLINE='quiet splash intel_iommu=off iommu=off video.brightness_switch_enabled=0 preempt=full isolcpus=domain,managed_irq,2,3 nohz_full=2,3 rcu_nocbs=2,3 irqaffinity=0-1,4-31 threadirqs nmi_watchdog=0 skew_tick=1'
log(){ echo "== $*"; }

log "1/10 apt holds (kernel/nvidia)"
apt-mark hold linux-image-generic-hwe-22.04 linux-headers-generic-hwe-22.04 linux-generic-hwe-22.04 \
  linux-image-$KVER linux-modules-$KVER linux-modules-extra-$KVER linux-headers-$KVER nvidia-dkms-580 nvidia-driver-580
log "2/10 tools: ethtool rt-tests"
DEBIAN_FRONTEND=noninteractive apt-get install -y ethtool rt-tests
log "3/10 groups realtime + docker for $TARGET_USER"
getent group realtime >/dev/null || groupadd realtime
usermod -aG realtime,docker "$TARGET_USER"
log "4/10 limits"
install -m 0644 "$SRC/security/limits.d/99-realtime.conf" /etc/security/limits.d/99-realtime.conf
log "5/10 GRUB: pin $KVER + RT cmdline"
grep -q "menuentry 'Ubuntu, with Linux $KVER'" /boot/grub/grub.cfg || { echo "GRUB entry for $KVER not found — run: grep menuentry /boot/grub/grub.cfg" >&2; exit 1; }
[ -f /etc/default/grub.bak-franka ] || cp /etc/default/grub /etc/default/grub.bak-franka
sed -i -E "s|^GRUB_DEFAULT=.*|GRUB_DEFAULT=\"gnulinux-advanced-$UUID>gnulinux-$KVER-advanced-$UUID\"|" /etc/default/grub
sed -i -E "s|^GRUB_CMDLINE_LINUX_DEFAULT=.*|GRUB_CMDLINE_LINUX_DEFAULT=\"$CMDLINE\"|" /etc/default/grub
update-grub
grep -q "isolcpus=domain,managed_irq,2,3" /boot/grub/grub.cfg
grep -q "^[[:space:]]*set default=\"gnulinux-advanced-$UUID>gnulinux-$KVER-advanced-$UUID\"" /boot/grub/grub.cfg   # grub-mkconfig indents this line
log "6/10 unattended-upgrades blacklist"
install -m 0644 "$SRC/apt/apt.conf.d/51franka-blacklist" /etc/apt/apt.conf.d/51franka-blacklist
apt-config dump | grep -q 'Package-Blacklist:: "linux-"'
log "7/10 r8126 module options + initramfs"
install -m 0644 "$SRC/modprobe.d/r8126.conf" /etc/modprobe.d/r8126.conf
update-initramfs -u -k "$KVER"
log "8/10 irqbalance off"
systemctl disable --now irqbalance || true
log "9/10 logind lid/idle + mask sleep targets"
install -d /etc/systemd/logind.conf.d
install -m 0644 "$SRC/systemd/logind.conf.d/franka.conf" /etc/systemd/logind.conf.d/franka.conf
systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
log "10/10 tmpfiles"
install -m 0644 "$SRC/tmpfiles.d/franka.conf" /etc/tmpfiles.d/franka.conf
systemd-tmpfiles --create /etc/tmpfiles.d/franka.conf
echo "DONE. Now: gsettings (as user, see A3 step 5), then: sudo reboot"
