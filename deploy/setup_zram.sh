#!/bin/sh
# Give the Pi Zero W compressed RAM swap (zram) so the kernel always has
# headroom for the Wi-Fi (brcmfmac) firmware buffers.  Without swap, memory
# pressure makes Wi-Fi fail to allocate and die until a power cycle.
#
# Run with:  sudo sh deploy/setup_zram.sh
set -eu

if [ "$(id -u)" -ne 0 ]; then
    echo "Run me with sudo: sudo sh deploy/setup_zram.sh" >&2
    exit 1
fi

if command -v zramctl >/dev/null 2>&1 && [ -f /etc/default/zramswap ]; then
    # zram-tools package present: configure it (persistent across reboots).
    grep -q '^ALGO=' /etc/default/zramswap || echo 'ALGO=lz4' >> /etc/default/zramswap
    sed -i 's/^ALGO=.*/ALGO=lz4/' /etc/default/zramswap
    grep -q '^PERCENT=' /etc/default/zramswap || echo 'PERCENT=50' >> /etc/default/zramswap
    sed -i 's/^PERCENT=.*/PERCENT=50/' /etc/default/zramswap
    systemctl enable zramswap 2>/dev/null || true
    systemctl restart zramswap
    echo "zram-tools configured: lz4, 50% of RAM"
else
    # Manual fallback (NOT persistent across reboot; see note at the end).
    modprobe zram
    echo lz4 > /sys/block/zram0/comp_algorithm 2>/dev/null || true
    echo 128M > /sys/block/zram0/disksize
    mkswap /dev/zram0 >/dev/null
    swapon -p 100 /dev/zram0
    echo "zram0 active at priority 100."
    echo "To persist: add 'swapon -p 100 /dev/zram0' to a boot script, or"
    echo "install zram-tools:  sudo apt-get install -y zram-tools"
fi

# Prefer moving pages to fast compressed swap over stalling allocations.
printf 'vm.swappiness=100\n' > /etc/sysctl.d/99-zram.conf
sysctl --system >/dev/null

free -h
