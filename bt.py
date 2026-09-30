import asyncio
import subprocess
from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.util import get_message_bus, Adapter

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
WIFI_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
IP_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"


def set_wifi_credentials(ssid, password):
    try:
        # List form (no shell) avoids breaking on quotes/special chars in SSID or password
        result = subprocess.run(
            ["sudo", "nmcli", "dev", "wifi", "connect", ssid, "password", password],
            capture_output=True, text=True, timeout=60,
        )
        print(f"[BLE] Wi-Fi Result: {result.stdout} {result.stderr}")
        return result.returncode == 0
    except Exception as e:
        print(f"[BLE Error] Wi-Fi setup failed: {e}")
        return False


def control_app(action):
    action_str = action.strip().upper()
    if action_str == "START":
        subprocess.run(["sudo", "systemctl", "start", "rover.service"])
        print("[BLE] Started rover.service")
    elif action_str == "STOP":
        subprocess.run(["sudo", "systemctl", "stop", "rover.service"])
        print("[BLE] Stopped rover.service")


def get_ip_address():
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5).stdout.split()
        ipv4 = [ip for ip in out if "." in ip]
        return ipv4[0] if ipv4 else ""
    except Exception:
        return ""


class RoverService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, True)

    @characteristic(WIFI_CHAR_UUID, CharacteristicFlags.WRITE)
    def wifi_char(self, options):
        return b""  # write-only, getter must still exist

    @wifi_char.setter
    def wifi_char(self, value, options):
        payload = bytes(value).decode("utf-8").strip()
        print("[BLE] Received Wi-Fi payload")
        if ":" in payload:
            ssid, password = payload.split(":", 1)
            set_wifi_credentials(ssid, password)

    @characteristic(CONTROL_CHAR_UUID, CharacteristicFlags.WRITE)
    def control_char(self, options):
        return b""

    @control_char.setter
    def control_char(self, value, options):
        command = bytes(value).decode("utf-8").strip()
        print(f"[BLE] Received App Command: {command}")
        control_app(command)
        
    @characteristic(IP_CHAR_UUID, CharacteristicFlags.READ)
    def ip_char(self, options):
        return get_ip_address().encode("utf-8")


async def main():
    bus = await get_message_bus()

    service = RoverService()
    await service.register(bus)

    adapter = await Adapter.get_first(bus)

    advert = Advertisement(
        localName="Smart-Rover",
        serviceUUIDs=[SERVICE_UUID],
        appearance=0x0000,
        timeout=0,
    )
    await advert.register(bus, adapter)

    print("[BLE] Rover GATT Service & Advertisement active and advertising!")
    await asyncio.Event().wait()


if __name__ == "__main__":
    asyncio.run(main())