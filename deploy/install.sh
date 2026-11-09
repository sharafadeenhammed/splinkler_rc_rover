# #!/usr/bin/env bash
# One-shot provisioning for the rover dashboard (Raspberry Pi OS Bookworm / Trixie).
#
#   bash deploy/install.sh                  # packages, venv, systemd units
#   bash deploy/install.sh --no-services    # skip the systemd units
#   WIFI_COUNTRY=NG bash deploy/install.sh  # also set the Wi-Fi country
#                                           # (a missing country leaves the radios soft-blocked)
#
# Run it as your normal user (rover_dev), NOT with sudo.  The script calls sudo
# itself where needed, so .venv and the project files stay user-owned.
#
# Safe to re-run: apt and pip are idempotent, the units are re-installed and
# re-enabled.  Reboot once after the first run if the boot-config check below
# reports I2C / DHT11 items.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$PROJECT_DIR/.venv"
SERVICE_DIR="/etc/systemd/system"
INSTALL_SERVICES=1
case "${1:-}" in
  --no-services) INSTALL_SERVICES=0 ;;
  "" ) ;;
  *) echo "usage: bash deploy/install.sh [--no-services]" >&2; exit 2 ;;
esac

# Running under sudo would create .venv as root even though the units run as
# rover_dev, so refuse and let the script call sudo only where it must.
if [ "$(id -u)" -eq 0 ]; then
  echo "install.sh: run as your normal user (not root / sudo)." >&2
  exit 1
fi
APP_USER="$(id -un)"

if ! command -v apt-get >/dev/null 2>&1; then
  echo "install.sh: this targets Raspberry Pi OS / Debian (apt-get not found)" >&2
  exit 1
fi

echo "==> project : $PROJECT_DIR"
echo "==> user    : $APP_USER"
if [ "$APP_USER" != "rover_dev" ]; then
  echo "    note: deploy/rover.service and deploy/mediamtx.service run as"
  echo "          rover_dev; edit User= in both units to match '$APP_USER'."
fi

# A fresh OS with no Wi-Fi country leaves Wi-Fi/Bluetooth soft-blocked.
if [ -n "${WIFI_COUNTRY:-}" ] && command -v raspi-config >/dev/null 2>&1; then
  echo "==> Wi-Fi country: $WIFI_COUNTRY"
  sudo raspi-config nonint do_wifi_country "$WIFI_COUNTRY" || echo "    could not set the Wi-Fi country"
fi

echo "==> system packages"
sudo apt-get update
# git         : the dashboard's "Update rover code" runs git pull
# python3-venv: creates .venv
# i2c-tools   : i2cdetect, for checking the MPU6050 / compass wiring
# rpicam-apps : rpicam-vid, which MediaMTX's rpiCamera source feeds from
sudo apt-get install -y --no-install-recommends \
  git bluez rfkill python3-venv python3-pip i2c-tools rpicam-apps

# pigpio is installed on its own: some OS releases don't ship it, and with
# set -e a missing package in the list above would abort the whole script.
echo "==> pigpio"
if ! sudo apt-get install -y --no-install-recommends pigpio; then
  echo "    WARNING: apt could not install 'pigpio' on this OS release."
  echo "             Build it from source and add a pigpiod.service, then re-run."
fi

# pigpiod must be up before the app starts: it owns the PWM timing for the
# L298N enables and the servo pulses, and it is what keeps driving the motors
# if the app is killed (deploy/emergency_stop.py talks to it directly).
if systemctl cat pigpiod.service >/dev/null 2>&1; then
  sudo systemctl enable --now pigpiod.service
  if ! systemctl is-active --quiet pigpiod.service; then
    echo "install.sh: pigpiod failed to start (journalctl -u pigpiod)" >&2
    exit 1
  fi
else
  echo "    pigpiod.service not found: motors, servos and encoders will not work until it is installed."
fi

# bluez is what bt.py talks to over D-Bus (BLE provisioning: Wi-Fi setup,
# START/STOP/RESTART_ROVER/RESTART_MEDIAMTX/UPDATE commands, IP and update status).
echo "==> bluetooth"
sudo systemctl enable --now bluetooth.service || echo "    bluetooth.service could not be started"
if command -v rfkill >/dev/null 2>&1; then
  RFKILL_BT="$(rfkill list bluetooth 2>/dev/null || true)"
  if grep -q 'Soft blocked: yes' <<<"$RFKILL_BT"; then
    sudo rfkill unblock bluetooth && echo "    unblocked the Bluetooth radio"
  fi
fi

# /dev/serial0 (GPS), /dev/video* (camera), i2c-dev and the BlueZ D-Bus API
# need real group membership; pigpio itself needs none because it goes through
# the daemon.
echo "==> groups for $APP_USER"
for grp in dialout video i2c gpio bluetooth; do
  getent group "$grp" >/dev/null || { echo "    (group $grp does not exist, skipped)"; continue; }
  if ! id -nG "$APP_USER" | tr ' ' '\n' | grep -qx "$grp"; then
    sudo usermod -aG "$grp" "$APP_USER" && echo "    added $grp"
  fi
done

echo "==> virtualenv: $VENV_DIR"
if [ ! -x "$VENV_DIR/bin/python" ]; then
  python3 -m venv "$VENV_DIR"
fi
"$VENV_DIR/bin/python" -m pip install --upgrade pip >/dev/null

# Keep in sync with the imports in main.py and helpers/.  gpiozero is
# deliberately pip-only: the distro python3-gpiozero package is older and gets
# shadowed inside the venv anyway.
#   flask / flask-sock  web app + the WebSocket control channel
#                       (flask-sock upgrades through simple-websocket)
#   gpiozero + pigpio   motors, servos, pump, moisture sensor
#   pyserial / pynmea2  GPS on /dev/serial0
#   smbus2              MPU6050 gyro and magnetometer over I2C
#   bluez-peripheral    BLE GATT peripheral used by bt.py (pure-Python via
#                       dbus-next, so no build toolchain needed)
#   gunicorn            optional WSGI server (deploy/gunicorn.conf.py)
echo "==> python packages"
"$VENV_DIR/bin/pip" install --upgrade \
  flask flask-sock simple-websocket \
  gpiozero pigpio \
  pyserial pynmea2 \
  smbus2 \
  bluez-peripheral \
  gunicorn

if [ "$INSTALL_SERVICES" -eq 1 ]; then
  echo "==> systemd units"
  sudo install -m 0644 "$PROJECT_DIR/deploy/rover.service"    "$SERVICE_DIR/rover.service"
  sudo install -m 0644 "$PROJECT_DIR/deploy/rover-bt.service" "$SERVICE_DIR/rover-bt.service"
  sudo systemctl daemon-reload

  # Catches mistakes such as two ExecStart= lines before they bite at runtime.
  sudo systemd-analyze verify "$SERVICE_DIR/rover.service" "$SERVICE_DIR/rover-bt.service" \
    || echo "    WARNING: systemd-analyze reported problems with a unit (see above)"

  # rover-bt.service must run at every boot: it is how you reach the Pi over
  # BLE when Wi-Fi is down.  rover.service is started from the dashboard
  # (START/STOP), so it is not enabled here; uncomment to start it at boot.
  # sudo systemctl enable rover.service
  sudo systemctl enable rover-bt.service
  sudo systemctl restart rover-bt.service   # pick up new bt.py on re-runs
  sleep 3
  systemctl --no-pager --lines=3 status rover-bt.service || true
fi

if [ ! -x "$PROJECT_DIR/bin/mediamtx" ]; then
  echo "==> camera: MediaMTX not installed yet — run 'bash deploy/setup_webrtc.sh'"
fi

# The kernel-side interfaces the app expects.  Nothing here is fatal: the
# helpers fail soft, so report and let the operator decide.
echo "==> boot config"
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt
if command -v raspi-config >/dev/null 2>&1 && [ "$(sudo raspi-config nonint get_i2c || echo 1)" != "0" ]; then
  echo "    I2C is OFF: 'sudo raspi-config nonint do_i2c 0' (MPU6050 + compass)"
fi
if [ -f "$CONFIG_TXT" ] && ! grep -q '^[[:space:]]*dtoverlay=dht11' "$CONFIG_TXT"; then
  echo "    DHT11 overlay missing: add 'dtoverlay=dht11,gpiopin=21' to $CONFIG_TXT"
fi
if grep -q 'console=serial0' /boot/firmware/cmdline.txt 2>/dev/null \
   || grep -q 'console=serial0' /boot/cmdline.txt 2>/dev/null; then
  echo "    serial console is enabled and shares /dev/serial0 with the GPS"
fi

cat <<EOF

Done.

  dashboard : http://$(hostname -I | awk '{print $1}'):5000
  logs      : journalctl -u rover -f      (BLE: journalctl -u rover-bt -f)
  camera    : bash deploy/setup_webrtc.sh   (WebRTC via MediaMTX)
  reboot    : required after I2C / DHT11 boot-config changes
EOF
