#!/usr/bin/env bash
#
# start_tailscale_funnel.sh
#
# Exposes the Go2 Remote Connection backend (FastAPI on :8000) to the public
# internet over HTTPS using Tailscale Funnel. This lets you open the website
# from ANY device/browser (phone, etc.) with nothing installed on it.
#
# Run this ON THE GO2 (the same machine that runs start_remote_connection.sh).
# Funnel config is stored by tailscaled and is restored on every boot, so you
# normally only run this ONCE. After that, the public URL works whenever the
# Go2 is powered on and the backend service is running.
#
# Public URL form:  https://<this-node>.<tailnet>.ts.net   (e.g.
#                   https://unitree-jetson-payload.tail85e7d7.ts.net )
#
# Usage:
#   ./start_tailscale_funnel.sh           # expose local :8000 (default)
#   ./start_tailscale_funnel.sh 8000      # expose a specific local port
#   ./start_tailscale_funnel.sh status    # show current funnel config + URL
#   ./start_tailscale_funnel.sh off       # stop funnel (tailscale funnel reset)
#
# Author: Victor Lim

set -euo pipefail

ARG="${1:-8000}"

if ! command -v tailscale >/dev/null 2>&1; then
  echo "❌ tailscale is not installed on this machine."
  echo "   Install:  https://tailscale.com/download/linux"
  echo "   Then:     sudo tailscale up"
  exit 1
fi

case "$ARG" in
  status)
    tailscale funnel status
    exit 0
    ;;
  off|reset|stop)
    echo "🛑 Disabling Tailscale Funnel..."
    tailscale funnel reset
    echo "✅ Funnel disabled (the website is no longer reachable from the internet)."
    exit 0
    ;;
esac

PORT="$ARG"

# Tailscale must be connected (logged in) for Funnel to work.
if ! tailscale status >/dev/null 2>&1; then
  echo "❌ Tailscale is installed but not connected. Run:  sudo tailscale up"
  exit 1
fi

# Best-effort: derive this node's public URL for a friendly message.
NODE_DNS=""
if command -v python3 >/dev/null 2>&1; then
  NODE_DNS="$(tailscale status --json 2>/dev/null \
    | python3 -c 'import sys,json; print(json.load(sys.stdin)["Self"]["DNSName"].rstrip("."))' 2>/dev/null || true)"
fi
URL="${NODE_DNS:+https://$NODE_DNS}"

# Soft warning if the backend is not listening yet (Funnel would return 502).
if ! curl -fsS -m 3 "http://127.0.0.1:${PORT}/health" >/dev/null 2>&1; then
  echo "⚠️  Nothing is answering on http://127.0.0.1:${PORT}/health yet."
  echo "    Funnel will still be set up, but it returns 502 until the backend runs."
  echo "    Start the backend with:  ./start_remote_connection.sh joystick"
  echo "    (or enable the systemd service so it starts on boot — see README §2)"
  echo
fi

echo "🌐 Exposing local port ${PORT} to the internet via Tailscale Funnel..."
[ -n "$NODE_DNS" ] && echo "   Node: ${NODE_DNS}"

# --bg : run in background and PERSIST across reboots (stored in tailscaled).
# --yes: don't prompt interactively (we're confirming public exposure on purpose).
if ! tailscale funnel --bg --yes "${PORT}"; then
  echo
  echo "❌ Funnel failed to start. The two usual one-time prerequisites:"
  echo "   1) Enable HTTPS certificates for the tailnet:"
  echo "        https://login.tailscale.com/admin/dns  ->  'HTTPS Certificates' -> Enable"
  echo "   2) Allow Funnel for this node in your tailnet policy (ACLs / nodeAttrs):"
  echo "        https://tailscale.com/kb/1223/funnel#requirements"
  echo "   Then re-run this script."
  exit 1
fi

echo
echo "✅ Funnel is live and will resume automatically on every reboot."
if [ -n "$URL" ]; then
  echo "   Open from any device/browser (no app needed):"
  echo "       ${URL}/"
  echo
  echo "   Health check:   curl ${URL}/health"
fi
echo
echo "   Inspect:  ./start_tailscale_funnel.sh status"
echo "   Stop:     ./start_tailscale_funnel.sh off"
