#!/usr/bin/env bash
# Install the robot's boot services (run on the Go2's Jetson):
#   sudo bash install_boot_services.sh
#
#   1. go2-remote-connection.service  <- go2-remote-connection_service.txt
#      (RL_start_remote_connection.sh with the stepfield map), enabled at boot.
#   2. /etc/default/vip-rescue-split  drops `realsense` from the VIP split stack's
#      boot list, so the vip-realsense container no longer takes the D435i.
#      livox + zenoh stay.
#
# Nothing is started, stopped or rebooted: both take effect at the next boot.
# Previous files are kept beside the new ones as *.bak-<date>.
set -euo pipefail
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT=/etc/systemd/system/go2-remote-connection.service
VIP_DEFAULT=/etc/default/vip-rescue-split
STAMP="$(date +%Y%m%d-%H%M%S)"

[ -f "$UNIT" ] && cp -p "$UNIT" "$UNIT.bak-$STAMP"
install -m 644 "$HERE/go2-remote-connection_service.txt" "$UNIT"
systemd-analyze verify "$UNIT" 2>&1 | grep -i "go2-remote-connection" || true

if systemctl cat vip-rescue-split.service >/dev/null 2>&1; then
  [ -f "$VIP_DEFAULT" ] && cp -p "$VIP_DEFAULT" "$VIP_DEFAULT.bak-$STAMP"
  printf '%s\n' \
    '# Boot list for vip-rescue-split.service (overrides the unit default' \
    '# "livox realsense zenoh"). realsense is left out: the D435i belongs to' \
    '# go2-remote-connection.service (RL_start_remote_connection.sh).' \
    'VIP_SPLIT_SERVICES="livox zenoh"' > "$VIP_DEFAULT"
  echo "wrote $VIP_DEFAULT"
else
  echo "vip-rescue-split.service not installed here: skipped $VIP_DEFAULT"
fi

systemctl daemon-reload
systemctl enable go2-remote-connection.service
echo
echo "Installed. Takes effect at the next boot (nothing was started or stopped)."
echo "  start now : sudo systemctl start go2-remote-connection   (journalctl -u go2-remote-connection -f)"
echo "  undo      : sudo systemctl disable go2-remote-connection; sudo rm $VIP_DEFAULT"
