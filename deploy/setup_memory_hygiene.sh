#!/bin/sh
# One-shot memory hygiene for the 512 MB Pi Zero W.  Idempotent — safe to
# re-run.  Applies: journald cap, ModemManager off, zram swap, Wi-Fi
# min-free-kbytes reservation, MemoryMax trim + daemon-reload.
#
# Run with:  sudo sh deploy/setup_memory_hygiene.sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "Run me with sudo: sudo sh deploy/setup_memory_hygiene.sh" >&2
    exit 1
fi

here="$(cd "$(dirname "$0")" && pwd)"

echo "== 1/5 journald cap (64M) =="
mkdir -p /etc/systemd/journald.conf.d
cp "$here/10-journal-size.conf" /etc/systemd/journald.conf.d/10-journal-size.conf
systemctl restart systemd-journald
journalctl --vacuum-size=64M >/dev/null 2>&1 || true
journalctl --disk-usage

echo "== 2/5 disable ModemManager (no modem on a Pi Zero W; ~10 MB RSS) =="
if systemctl list-unit-files ModemManager.service >/dev/null 2>&1; then
    systemctl disable --now ModemManager 2>/dev/null || true
fi

echo "== 3/5 zram compressed swap (headroom so brcmfmac never starves) =="
sh "$here/setup_zram.sh"

echo "== 4/5 Wi-Fi reserve: vm.min_free_kbytes=16384 =="
# Keep 16 MB free at all times so the brcmfmac firmware can allocate its
# buffers at interrupt time even when user-space is pressing on memory.
printf 'vm.min_free_kbytes=16384\n' > /etc/sysctl.d/99-wifi-reserve.conf
sysctl --system >/dev/null

echo "== 5/5 rover.service MemoryMax 300M -> 220M =="
unit=/etc/systemd/system/rover.service
if [ -f "$unit" ]; then
    sed -i 's/^MemoryMax=.*/MemoryMax=220M/' "$unit"
    systemctl daemon-reload
    grep -n '^MemoryMax=' "$unit"
else
    echo "rover.service not installed at $unit (skip — deploy the unit first)"
fi

echo "== done =="
free -h
