# import asyncio
# import subprocess
# import threading
# from bluez_peripheral.gatt.service import Service
# from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags
# from bluez_peripheral.advert import Advertisement
# from bluez_peripheral.util import get_message_bus, Adapter



# SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
# WIFI_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
# CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
# IP_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"
# UPDATE_STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef4"

# PROJECT_DIR = "/home/rover_dev/project"   # must be a git repo
# update_status = "IDLE"

# def set_wifi_credentials(ssid, password):
#     try:
#         # List form (no shell) avoids breaking on quotes/special chars in SSID or password
#         result = subprocess.run(
#             ["sudo", "nmcli", "dev", "wifi", "connect", ssid, "password", password],
#             capture_output=True, text=True, timeout=60,
#         )
#         print(f"[BLE] Wi-Fi Result: {result.stdout} {result.stderr}")
#         return result.returncode == 0
#     except Exception as e:
#         print(f"[BLE Error] Wi-Fi setup failed: {e}")
#         return False



# def control_app(action):
#     global update_status
#     action_str = action.strip().upper()
#     if action_str == "START":
#         subprocess.run(["sudo", "systemctl", "start", "rover.service"])
#         print("[BLE] Started rover.service")
#     elif action_str == "STOP":
#         subprocess.run(["sudo", "systemctl", "stop", "rover.service"])
#         print("[BLE] Stopped rover.service")
#     elif action_str == "UPDATE":
#         if update_status != "RUNNING":
#             update_status = "RUNNING"   # set now so the dashboard never reads a stale status
#             threading.Thread(target=run_update, daemon=True).start()
#             print("[BLE] Update started")

# def get_ip_address():
#     try:
#         out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5).stdout.split()
#         ipv4 = [ip for ip in out if "." in ip]
#         return ipv4[0] if ipv4 else ""
#     except Exception:
#         return ""

# def run_update():
#     global update_status
#     try:
#         r = subprocess.run(["git", "-C", PROJECT_DIR, "pull", "--ff-only"],
#                            capture_output=True, text=True, timeout=120)
#         if r.returncode == 0:
#             if "Already up to date" in r.stdout:
#                 update_status = "OK: already up to date"
#             else:
#                 update_status = "OK: updated. Stop/Start the app to apply."
#         else:
#             lines = (r.stderr.strip() or r.stdout.strip()).splitlines()
#             update_status = "FAIL: " + (lines[-1] if lines else "git pull failed")[:150]
#     except Exception as e:
#         update_status = f"FAIL: {e}"[:160]

# class RoverService(Service):
#     def __init__(self):
#         super().__init__(SERVICE_UUID, True)

#     @characteristic(WIFI_CHAR_UUID, CharacteristicFlags.WRITE)
#     def wifi_char(self, options):
#         return b""  # write-only, getter must still exist

#     @wifi_char.setter
#     def wifi_char(self, value, options):
#         payload = bytes(value).decode("utf-8").strip()
#         print("[BLE] Received Wi-Fi payload")
#         if ":" in payload:
#             ssid, password = payload.split(":", 1)
#             set_wifi_credentials(ssid, password)

#     @characteristic(CONTROL_CHAR_UUID, CharacteristicFlags.WRITE)
#     def control_char(self, options):
#         return b""

#     @control_char.setter
#     def control_char(self, value, options):
#         command = bytes(value).decode("utf-8").strip()
#         print(f"[BLE] Received App Command: {command}")
#         control_app(command)
        
#     @characteristic(IP_CHAR_UUID, CharacteristicFlags.READ)
#     def ip_char(self, options):
#         return get_ip_address().encode("utf-8")
    
#     @characteristic(UPDATE_STATUS_CHAR_UUID, CharacteristicFlags.READ)
#     def update_status_char(self, options):
#         return update_status.encode("utf-8")


# async def get_adapter(bus):
#     """Pick the Bluetooth adapter (hci0, hci1, ...) directly.

#     Adapter.get_first() treats every node under /org/bluez as an adapter and
#     crashes with InterfaceNotFoundError when a non-adapter node such as
#     /org/bluez/test exists. Here we only look at nodes named hciX.
#     """
#     introspection = await bus.introspect("org.bluez", "/org/bluez")
#     names = sorted(n.name for n in introspection.nodes if n.name.startswith("hci"))
#     if not names:
#         raise RuntimeError("No Bluetooth adapter (hciX) found under /org/bluez")

#     path = f"/org/bluez/{names[0]}"
#     adapter_introspection = await bus.introspect("org.bluez", path)
#     proxy = bus.get_proxy_object("org.bluez", path, adapter_introspection)
#     return Adapter(proxy)


# async def main():
#     bus = await get_message_bus()

#     adapter = await get_adapter(bus)

#     service = RoverService()
#     await service.register(bus, adapter=adapter)

#     advert = Advertisement(
#         localName="Smart-Rover",
#         serviceUUIDs=[SERVICE_UUID],
#         appearance=0x0000,
#         timeout=0,
#     )
#     await advert.register(bus, adapter)

#     print("[BLE] Rover GATT Service & Advertisement active and advertising!")
#     await asyncio.Event().wait()


# if __name__ == "__main__":
#     asyncio.run(main())





import asyncio
import subprocess
import threading
from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import characteristic, CharacteristicFlags
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.util import get_message_bus, Adapter



SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
WIFI_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
IP_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"
UPDATE_STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef4"

PROJECT_DIR = "/home/rover_dev/project"   # must be a git repo
APP_USER = "rover_dev"                    # owner of PROJECT_DIR (git runs as this user)
update_status = "IDLE"


def run_in_thread(fn, *args):
    """Run slow work (nmcli, systemctl, git) off the BLE event loop.

    The GATT write handlers run inside the asyncio loop that also answers
    BlueZ over D-Bus. If a handler blocks (nmcli can take 10-60 s), BLE stops
    responding, the write times out and the phone/PC drops the connection.
    """
    threading.Thread(target=fn, args=args, daemon=True).start()


def set_wifi_credentials(ssid, password):
    try:
        # List form (no shell) avoids breaking on quotes/special chars in SSID or password
        result = subprocess.run(
            ["sudo", "nmcli", "dev", "wifi", "connect", ssid, "password", password],
            capture_output=True, text=True, timeout=60,
        )
        print(f"[BLE] Wi-Fi Result (code {result.returncode}): {result.stdout} {result.stderr}")
        return result.returncode == 0
    except Exception as e:
        print(f"[BLE Error] Wi-Fi setup failed: {e}")
        return False


def systemctl_action(verb):
    try:
        r = subprocess.run(["sudo", "systemctl", verb, "rover.service"],
                           capture_output=True, text=True, timeout=30)
        print(f"[BLE] systemctl {verb} rover.service -> code {r.returncode} {r.stderr.strip()}")
    except Exception as e:
        print(f"[BLE Error] systemctl {verb} failed: {e}")


def control_app(action):
    global update_status
    action_str = action.strip().upper()
    if action_str == "START":
        run_in_thread(systemctl_action, "start")
    elif action_str == "STOP":
        run_in_thread(systemctl_action, "stop")
    elif action_str == "UPDATE":
        if update_status != "RUNNING":
            update_status = "RUNNING"   # set now so the dashboard never reads a stale status
            run_in_thread(run_update)
            print("[BLE] Update started")
    else:
        print(f"[BLE] Unknown command: {action_str}")


def get_ip_address():
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=5).stdout.split()
        ipv4 = [ip for ip in out if "." in ip]
        return ipv4[0] if ipv4 else ""
    except Exception:
        return ""


def run_update():
    global update_status
    try:
        # Run git as the repo's owner. As root, git refuses with
        # "detected dubious ownership" and would also create root-owned files.
        r = subprocess.run(
            ["sudo", "-u", APP_USER, "git", "-C", PROJECT_DIR, "pull", "--ff-only"],
            capture_output=True, text=True, timeout=120,
        )
        if r.returncode == 0:
            if "Already up to date" in r.stdout:
                update_status = "OK: already up to date"
            else:
                update_status = "OK: updated. Stop/Start the app to apply."
        else:
            lines = (r.stderr.strip() or r.stdout.strip()).splitlines()
            update_status = "FAIL: " + (lines[-1] if lines else "git pull failed")[:150]
    except Exception as e:
        update_status = f"FAIL: {e}"[:160]


class RoverService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, True)

    @characteristic(WIFI_CHAR_UUID, CharacteristicFlags.WRITE)
    def wifi_char(self, options):
        return b""  # write-only, getter must still exist

    @wifi_char.setter
    def wifi_char(self, value, options):
        # Only strip newlines so passwords with leading/trailing spaces survive
        payload = bytes(value).decode("utf-8").rstrip("\r\n")
        print("[BLE] Received Wi-Fi payload")
        if ":" in payload:
            ssid, password = payload.split(":", 1)
            run_in_thread(set_wifi_credentials, ssid, password)
        else:
            print("[BLE] Wi-Fi payload has no ':' separator, ignoring")

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

    @characteristic(UPDATE_STATUS_CHAR_UUID, CharacteristicFlags.READ)
    def update_status_char(self, options):
        return update_status.encode("utf-8")


async def get_adapter(bus):
    """Pick the Bluetooth adapter (hci0, hci1, ...) directly.

    Adapter.get_first() treats every node under /org/bluez as an adapter and
    crashes with InterfaceNotFoundError when a non-adapter node such as
    /org/bluez/test exists. Here we only look at nodes named hciX.
    """
    introspection = await bus.introspect("org.bluez", "/org/bluez")
    names = sorted(n.name for n in introspection.nodes if n.name.startswith("hci"))
    if not names:
        raise RuntimeError("No Bluetooth adapter (hciX) found under /org/bluez")

    path = f"/org/bluez/{names[0]}"
    adapter_introspection = await bus.introspect("org.bluez", path)
    proxy = bus.get_proxy_object("org.bluez", path, adapter_introspection)
    return Adapter(proxy)


async def main():
    bus = await get_message_bus()

    adapter = await get_adapter(bus)

    service = RoverService()
    await service.register(bus, adapter=adapter)

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