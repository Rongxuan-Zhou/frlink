#!/bin/bash
# franka-docker-user.sh — insert DROP rules into Docker's DOCKER-USER chain so container traffic is never forwarded via the
# robot NIC or the rog link (belt-and-braces to the nftables forward chain). Idempotent; ip-forward stays on for other containers.
set -u
for i in $(seq 1 30); do iptables -S DOCKER-USER >/dev/null 2>&1 && break; sleep 1; done
if ! iptables -S DOCKER-USER >/dev/null 2>&1; then
  echo "franka-docker-user: DOCKER-USER chain absent (docker iptables mode off?) — nftables forward drop still applies"; exit 0
fi
for ifn in enp110s0 frlink0; do
  for d in -i -o; do
    iptables -C DOCKER-USER $d "$ifn" -j DROP 2>/dev/null || iptables -I DOCKER-USER $d "$ifn" -j DROP
  done
done
iptables -S DOCKER-USER
