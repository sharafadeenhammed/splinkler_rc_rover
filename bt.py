import asyncio
import os
import subprocess
from bleak import BleakGATTCharacteristic, BleakServer

# Unique UUIDs for your Rover's BLE Service and Characteristics
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
WIFI_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"  # Write: SSID:Password
CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2" # Write: START / STOP
STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"  # Read: Current App Status

def set_wifi_credentials(ssid, password):
    """Uses NetworkManager (nmcli) or wpa_supplicant to connect to new Wi-Fi."""
    try:
        # NetworkManager approach (standard on modern Pi OS Bookworm)
        cmd = f"sudo nmcli dev wifi connect '{ssid}' password '{password}'"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            return True, "Connected successfully"
        else:
            return False, result.stderr
    except Exception as e:
        return False, str(e)

def control_app(action):
    """Starts or stops the rover systemd service."""
    if action.upper() == "START":
        os.system("sudo systemctl start rover.service")
        return "App Started"
    elif action.upper() == "STOP":
        os.system("sudo systemctl stop rover.service")
        return "App Stopped"
    return "Invalid Action"

def get_app_status():
    """Checks if rover.service is active."""
    result = subprocess.run("systemctl is-active rover.service", shell=True, capture_output=True, text=True)
    return result.stdout.strip()

# --- BLE Handlers ---

def write_wifi_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    try:
        # Expected payload format: "SSID:Password"
        payload = data.decode("utf-8").strip()
        if ":" in payload:
            ssid, password = payload.split(":", 1)
            print(f"[BLE] Provisioning Wi-Fi for SSID: {ssid}")
            success, msg = set_wifi_credentials(ssid, password)
            print(f"[BLE] Wi-Fi Result: {msg}")
    except Exception as e:
        print(f"[BLE] Error parsing Wi-Fi data: {e}")

def write_control_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    try:
        command = data.decode("utf-8").strip()
        print(f"[BLE] Received App Command: {command}")
        status = control_app(command)
        print(f"[BLE] {status}")
    except Exception as e:
        print(f"[BLE] Error processing command: {e}")

def read_status_handler(characteristic: BleakGATTCharacteristic) -> bytearray:
    status = get_app_status()
    return bytearray(f"Rover App: {status}".encode("utf-8"))

async def main():
    print("[BLE] Starting Rover Bluetooth Provisioning Agent...")
    
    # Configure BLE Server with callbacks
    server = BleakServer()
    # Note: Depending on your Bleak setup, you can advertise the service 
    # using BlueZ DBus interfaces or custom GATT servers.
    
    print("[BLE] Agent is running and waiting for connections...")
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())