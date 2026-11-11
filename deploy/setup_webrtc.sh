#!/usr/bin/env bash
# Install MediaMTX: Raspberry Pi camera -> hardware H.264 -> WebRTC (WHEP).
#
#   bash deploy/setup_webrtc.sh              # install + enable at boot
#   bash deploy/setup_webrtc.sh --no-service # binary only, no systemd unit
#
# Downloads a ~20 MB GitHub release (needs internet once on the Pi) and uses
# sudo only for the systemd unit. The binary lands in <project>/bin/, so the
# application folder stays self-contained.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN_DIR="$PROJECT_DIR/bin"
UNIT_SRC="$PROJECT_DIR/deploy/mediamtx.service"
UNIT_DST="/etc/systemd/system/mediamtx.service"
INSTALL_SERVICE=1
[[ "${1:-}" == "--no-service" ]] && INSTALL_SERVICE=0

case "$(uname -m)" in
  aarch64|arm64) ARCH=arm64 ;;
  armv7l|armv7)  ARCH=armv7 ;;
  armv6l|armv6)  ARCH=armv6 ;;   # Raspberry Pi Zero / Zero W
  *) echo "unsupported architecture: $(uname -m) — this needs a Raspberry Pi"; exit 1 ;;
esac

if ! command -v rpicam-vid >/dev/null 2>&1; then
  echo "warning: rpicam-vid not found — MediaMTX's rpiCamera source needs it."
  echo "         Install it with: sudo apt install -y rpicam-apps"
fi

echo "==> resolving latest MediaMTX release for linux_${ARCH}"
TAG="$(curl -fsSL https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
       | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p' | head -n1)"
if [ -z "$TAG" ]; then
  echo "could not resolve the release tag — is the Pi online?"
  exit 1
fi

ASSET="mediamtx_${TAG}_linux_${ARCH}.tar.gz"
URL="https://github.com/bluenviron/mediamtx/releases/download/${TAG}/${ASSET}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "==> downloading ${ASSET}"
curl -fL --progress-bar "$URL" -o "$TMP/$ASSET"
tar -xzf "$TMP/$ASSET" -C "$TMP"

mkdir -p "$BIN_DIR"
install -m 0755 "$TMP/mediamtx" "$BIN_DIR/mediamtx"
echo "==> installed ${BIN_DIR}/mediamtx (${TAG})"

if [ "$INSTALL_SERVICE" -eq 1 ]; then
  echo "==> installing systemd unit"
  sudo install -m 0644 "$UNIT_SRC" "$UNIT_DST"
  sudo systemctl daemon-reload
  sudo systemctl enable mediamtx.service >/dev/null
  sudo systemctl restart mediamtx.service
  sleep 2
  systemctl --no-pager --lines=5 status mediamtx.service || true
fi

LAN_IP="$(hostname -I | awk '{print $1}')"
cat <<EOF

Done — the dashboard camera tile now uses WebRTC (H.264), with no MJPEG fallback.

  WHEP endpoint : http://${LAN_IP}:8889/rover/whep
  Built-in page : http://${LAN_IP}:8889/rover
  RTSP (local)  : disabled — set 'rtsp: true' in deploy/mediamtx.yml to use it

Ports 8889/tcp (signalling) and 8189/udp (media) must be reachable from the
browser. Check logs with: journalctl -u mediamtx -f
EOF
