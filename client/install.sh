#!/bin/bash
# install.sh -- set up this PC as a frlink client. Idempotent: run it again after editing
# config.env. It never needs root; the two privileged steps (NIC profile, linger) are printed.
#
#   1. symlink bin/ into ~/.local/bin
#   2. create the forced-command ssh key if missing, print the authorized_keys line for the host admin
#   3. write the "Host <alias>" block into ~/.ssh/config (between markers, replaced on rerun)
#   4. add the RT host's key to known_hosts if the host is reachable (franka-remote uses
#      StrictHostKeyChecking=yes)
#   5. install + enable the franka-state-mirror user unit
#   6. print the nmcli command for the private-link profile
set -euo pipefail
ROOT=$(cd "$(dirname "$(readlink -f "$0")")" && pwd)

if [ ! -f "$ROOT/config.env" ]; then
  cp "$ROOT/config.env.example" "$ROOT/config.env"
  echo "created $ROOT/config.env from config.env.example."
  echo "Edit it (FRANKA_CTL_USER, FRANKA_LINK_IFACE, FRANKA_PY at least) and run install.sh again."
  exit 1
fi
. "$ROOT/config.env"
for v in FRANKA_SERVO_HOST FRANKA_CTL_USER FRANKA_CLIENT_IP FRANKA_LINK_IFACE; do
  [ -n "${!v:-}" ] || { echo "install.sh: $v is empty in config.env" >&2; exit 64; }
done
KEY=${FRANKA_CTL_KEY:-$HOME/.ssh/franka_ctl}
ALIAS=${FRANKA_CTL_SSH_ALIAS:-alienware-rt}
CTL_PATH=${FRANKA_CTL_PATH:-/home/$FRANKA_CTL_USER/franka/bin/franka-ctl}

echo "== 1. bin/ -> ~/.local/bin"
mkdir -p "$HOME/.local/bin"
for b in franka_state_mirror franka-remote franka-client-preflight franka-teleop; do
  ln -sfn "$ROOT/bin/$b" "$HOME/.local/bin/$b"
  echo "   $HOME/.local/bin/$b -> $ROOT/bin/$b"
done
chmod +x "$ROOT"/bin/franka* "$ROOT"/tests/*.py "$ROOT"/record/*.py "$ROOT"/teleop/*.py 2>/dev/null || true
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) echo "   note: $HOME/.local/bin is not in PATH for this shell";; esac

echo "== 2. ssh key $KEY"
mkdir -p "$HOME/.ssh"; chmod 700 "$HOME/.ssh"
if [ ! -f "$KEY" ]; then
  ssh-keygen -q -t ed25519 -N '' -f "$KEY" -C "franka-ctl@$(hostname -s)"
  echo "   generated"
else
  echo "   exists"
fi
chmod 600 "$KEY"

echo "== 3. ~/.ssh/config Host $ALIAS"
CFG=$HOME/.ssh/config
touch "$CFG"; chmod 600 "$CFG"
python3 - "$CFG" "$ALIAS" "$FRANKA_SERVO_HOST" "$FRANKA_CTL_USER" "$KEY" <<'EOF'
import re, sys
cfg, alias, host, user, key = sys.argv[1:]
block = f"""# >>> frlink client (managed by install.sh) >>>
Host {alias}
    HostName {host}
    User {user}
    IdentityFile {key}
    IdentitiesOnly yes
    BatchMode yes
    ConnectTimeout 3
# <<< frlink client <<<
"""
text = open(cfg).read()
pat = re.compile(r"# >>> frlink client.*?# <<< frlink client <<<\n", re.S)
new = pat.sub(block, text) if pat.search(text) else (text + ("\n" if text and not text.endswith("\n") else "") + block)
if new != text:
    open(cfg, "w").write(new)
    print("   written")
else:
    print("   unchanged")
EOF

echo "== 4. known_hosts entry for $FRANKA_SERVO_HOST"
KH=$HOME/.ssh/known_hosts; touch "$KH"
if ssh-keygen -F "$FRANKA_SERVO_HOST" -f "$KH" >/dev/null 2>&1; then
  echo "   present"
elif ping -c 1 -W 1 "$FRANKA_SERVO_HOST" >/dev/null 2>&1 && ssh-keyscan -T 3 -t ed25519 "$FRANKA_SERVO_HOST" 2>/dev/null | grep -q ssh-ed25519; then
  ssh-keyscan -T 3 -t ed25519 "$FRANKA_SERVO_HOST" 2>/dev/null >> "$KH"
  echo "   added by ssh-keyscan; verify the fingerprint with the host admin:"
  ssh-keygen -lf <(ssh-keyscan -T 3 -t ed25519 "$FRANKA_SERVO_HOST" 2>/dev/null) | sed 's/^/     /'
else
  echo "   host not reachable now; once the cable is up run:"
  echo "     ssh-keyscan -t ed25519 $FRANKA_SERVO_HOST >> ~/.ssh/known_hosts"
fi

echo "== 5. franka-state-mirror user unit"
UNIT_DIR=$HOME/.config/systemd/user
mkdir -p "$UNIT_DIR"
sed -e "s|@ROOT@|$ROOT|g" -e "s|@FRANKA_SERVO_HOST@|$FRANKA_SERVO_HOST|g" \
    "$ROOT/systemd/franka-state-mirror.service" > "$UNIT_DIR/franka-state-mirror.service"
if systemctl --user daemon-reload 2>/dev/null; then
  systemctl --user enable --now franka-state-mirror >/dev/null 2>&1 && echo "   enabled and started" || echo "   enable/start failed: systemctl --user status franka-state-mirror"
  systemctl --user restart franka-state-mirror 2>/dev/null || true
  if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != yes ]; then
    echo "   optional: 'loginctl enable-linger $USER' keeps the mirror running without a login session"
  fi
else
  echo "   systemd --user is not available in this session; unit written to $UNIT_DIR"
fi

echo
echo "== hand this line to the RT host admin (append to ~$FRANKA_CTL_USER/.ssh/authorized_keys on the host):"
echo
printf 'command="%s",no-port-forwarding,no-X11-forwarding,no-agent-forwarding,no-pty,from="%s" %s\n' \
  "$CTL_PATH" "${FRANKA_CLIENT_IP%.*}.0/24" "$(cat "$KEY.pub")"
echo
echo "== private link (needs NetworkManager privileges; run once, adjust the interface if needed):"
echo
echo "  nmcli con add type ethernet ifname $FRANKA_LINK_IFACE con-name franka-link \\"
echo "      ipv4.method manual ipv4.addresses $FRANKA_CLIENT_IP/24 ipv4.never-default yes \\"
echo "      ipv6.method disabled connection.autoconnect yes"
echo "  nmcli con up franka-link"
echo
echo "== then: franka-client-preflight"
