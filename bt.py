import asyncio
import os
import subprocess
from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags
from bluez_peripheral.advert.advertisement import Advertisement
from bluez_peripheral.agent import NoInputNoOutputAgent

# Unique UUIDs matching your app setup
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
WIFI_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"

def set_wifi_credentials(ssid, password):
    try:
        cmd = f"sudo nmcli dev wifi connect '{ssid}' password '{password}'"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode == 0
    except Exception as e:
        print(f"[BLE Error] Wi-Fi setup failed: {e}")
        return False

def control_app(action):
    action_str = action.strip().upper()
    if action_str == "START":
        os.system("sudo systemctl start rover.service")
        print("[BLE] Started rover.service")
    elif action_str == "STOP":
        os.system("sudo systemctl stop rover.service")
        print("[BLE] Stopped rover.service")

class RoverService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, True)

    @characteristic(WIFI_CHAR_UUID, CharacteristicFlags.WRITE)
    def wifi_char(self, options):
        pass

    @wifi_char.setter
    def wifi_char(self, value, options):
        payload = bytes(value).decode("utf-8").strip()
        print(f"[BLE] Received Wi-Fi payload: {payload}")
        if ":" in payload:
            ssid, password = payload.split(":", 1)
            set_wifi_credentials(ssid, password)

    @characteristic(CONTROL_CHAR_UUID, CharacteristicFlags.WRITE)
    def control_char(self, options):
        pass

    @control_char.setter
    def control_char(self, value, options):
        command = bytes(value).decode("utf-8").strip()
        print(f"[BLE] Received App Command: {command}")
        control_app(command)

async def main():
    bus = await asyncio.get_event_loop().run_in_executor(None, lambda: None)
    
    service = RoverService()
    await service.register()

    # Configure BLE Advertisement
    advert = Advertisement(
        local_name="Smart-Rover",
        service_uuids=[SERVICE_UUID],
        appearance=0x0000,
        timeout=0
    )
    await advert.register()

    print("[BLE] Rover GATT Service & Advertisement running!")
    
    # Keep running
    while True:
        await asyncio.sleep(3600)

if __name__ == "__main__":
    asyncio.run(main())