# Rover OS — Sprinkler Rover

A Raspberry Pi–controlled sprinkler rover: a small wheeled robot with a
soil-moisture probe, a water pump, a panning camera and a live browser
dashboard. Drive it manually over the LAN, watch the camera, read the sensors,
and deploy the probe or run the pump — all from one page. A separate Bluetooth
LE dashboard provisions Wi-Fi and manages the systemd services when the
network is unavailable.

Driving is manual by default, and an optional autonomous sweep can take over:
it runs parallel rows relative to the heading at Start (the rover never needs
to be pointed north), checks the soil every few metres and waters only when
the probe is dry. The map, pose and localization stack shows where the rover
is in either mode. In manual mode the ultrasonic sensor only reports distance
and does not stop the rover; the sweep uses it to pause for obstacles.

## Features

- **Manual drive** — hold-to-drive pad, WASD/arrow keys, speed slider (0–100%),
  space to stop. `C` centres the camera, `E`/`M` toggle the probe, `P` toggles
  the pump. Commands go over a WebSocket with an HTTP fallback.
- **Live video** — Pi camera → hardware H.264 → WebRTC (WHEP) through MediaMTX.
  The camera and encoder stop when nobody is watching.
- **Auto sweep** — bounded lawnmower coverage (`length_m` × `width_m` at
  `row_spacing_m`, turned to the left or right), soil checks every
  `water_interval_m`, pump only when the probe reads dry, gyro-closed turns,
  and obstacle wait/sidestep with retries. The dashboard's *Auto sweep* dock
  sets the plot and shows state, row and progress; its state is in telemetry.
- **Telemetry** at ~5 Hz — temperature/humidity, soil state, obstacle distance,
  encoder ticks, wheel duties, heading/attitude, GPS, local pose, memory.
- **Work map** — Leaflet + OpenStreetMap with rover position, heading and trail.
- **Equipment** — soil-probe deploy/retract servo, water pump, camera pan.
- **Safety failsafes** — the rover stops motors, pump and probe whenever the
  dashboard disconnects, the process exits, or the service is stopped.
- **BLE provisioning** — send Wi-Fi credentials, start/stop/restart the app and
  MediaMTX, pull or commit code over Git, read back the IP and update status.
- **Optional command PIN** to protect the drive/probe/pump/camera endpoints.

## Hardware

`constants/pinout.py` is the authoritative 40-pin ledger — check it before
wiring anything new. Summary:

| Component | Role | Connection |
|---|---|---|
| Raspberry Pi Zero W (512 MB) | Host | 40-pin header; any Pi works, `pigpiod` required |
| L298N dual H-bridge | Motor driver | ENA/ENB speed PWM, IN1–IN4 direction |
| 2× JGB37-520 gearmotors | Drive | Encoder channels A/B per wheel |
| MPU6050 | Gyro/accelerometer, relative yaw | I²C `0x68` |
| HMC5883L / QMC5883P | Magnetometer for drift correction | I²C `0x1E` or `0x2C` |
| NMEA GPS | Position and course | UART `/dev/serial0` @ 9600 |
| DHT11 | Temperature/humidity | kernel `dtoverlay=dht11,gpiopin=21` |
| HC-SR04 | Ultrasonic distance | GPIO trigger/echo |
| 2× servo | Camera pan and soil-probe arm | GPIO 23 and 26 |
| LM393 soil module | Wet/dry digital output | GPIO 24 |
| Water pump + transistor | Irrigation | GPIO 25 |

Notes:

- Remove the **ENA/ENB jumper caps** on the L298N. With the jumpers fitted the
  enables are tied high and the motors ignore the speed slider.
- The DHT11 is read through the kernel IIO driver, not bit-banged.
- `pigpiod` owns the PWM timing. The app and `deploy/emergency_stop.py` both
  assume it is running.

## Repository layout

```
main.py                    Flask app: HTTP API, WebSocket control, telemetry,
                           background sensor / navigation / broadcast threads
bt.py                      BLE GATT peripheral for provisioning (root)
index.html                 Standalone Web Bluetooth provisioning dashboard
templates/index.html       Rover dashboard shell
static/index.js            Dashboard logic: drive, map, WebRTC, telemetry
static/index.css           Dashboard styling
static/vendor/             Leaflet bundle
helpers/                   Hardware drivers and navigation maths
  auto_controller.py       Auto sweep thread: state machine, watering,
                           obstacle handling, start/stop/status API
  auto_plan.py             Pure row/turn targets, steering and sweep maths
  motor_controls.py        L298N drive, tank/arcade helpers
  encoder_controller.py    Encoder counting and wheel geometry
  mpu6050.py               Gyro attitude and relative heading
  hmc5883l.py              Magnetometer and compass drift correction
  gps_controller.py        NMEA reader (fails soft without hardware)
  pose_fusion.py           ENU conversions, GPS quality gates
  navigation_database.py   Pose tracker + persisted map origin
  ultrasonic_controller.py HC-SR04 driver
  dht11.py                 Kernel IIO temperature/humidity reader
  moisture_controller.py   Soil sensor state
  pump_controller.py       Water pump
  servo_controller.py      Camera and probe servos
constants/pinout.py        Pin ledger
deploy/                    Installer, systemd units, MediaMTX, failsafes
calibrate_wheels.py        Interactive direction/encoder calibration
test_fusion_logic.py       Hardware-free fusion tests
test_auto_plan.py          Hardware-free auto-mode planning tests
test_motor.py              Bench script (drives forward forever)
```

`old.main.py`, `image.png`, `sensors.txt` and `work_areas.json` are ignored by
Git and are not part of the app.

## How it works

### Web app

`main.py` is a single Flask process with `flask-sock`. Importing the module
starts three daemon threads, so both `python main.py` and a WSGI server get
telemetry without an HTTP request:

- **Sensor loop** — reads DHT11 about once per second.
- **Navigation loop** — 20 Hz pose update (encoders + IMU + compass + GPS).
- **Broadcast loop** — pushes telemetry to the WebSocket client at ~5 Hz.

Only one dashboard may control the rover at a time (`MAX_WS_CLIENTS = 1`). The
dashboard pings every 2 s; a socket silent for 10 s is dropped, which triggers
the failsafe. HTTP endpoints mirror every control command so a lost WebSocket
does not strand the operator. A sweep adds a fourth thread only while it runs,
so the rest of the app behaves identically in manual mode.

### Localization

The pose is local ENU metres (x east, y north) around a map origin:

1. Wheel encoders integrate distance and heading (differential odometry).
2. The MPU6050 yaw replaces the encoder heading when available; a still gate
   freezes the heading while parked and re-learns gyro bias.
3. The compass supplies heading *changes* (not absolute headings), so board
   mounting rotation and magnetic declination cancel out. Rotation sense is
   learned on the first real turn.
4. GPS is blended in slowly with a minimum-satellite gate, a speed gate for
   course, and spike rejection.

The map origin is stored atomically in `work_areas.json`. If none is saved, the
first GPS fix is adopted automatically while the rover is stopped. The IMU and
GPS are optional; without them the rover still drives and falls back to
odometry.

### Safety

- Dashboard disconnect or a silent socket → motors off, pump off, probe
  retracted (`emergency_stop_rover`).
- `atexit` and `SIGTERM` handlers cover Ctrl-C, `systemctl stop` and gunicorn's
  graceful shutdown.
- `deploy/emergency_stop.py` runs as `ExecStopPost`, zeroing the motor pins and
  pump directly through `pigpiod` even after an OOM kill or `SIGKILL`.
- With `ROVER_PIN` set, the four control groups (drive, probe, pump, camera pan)
  and starting a sweep require a PIN on both HTTP and WebSocket; stopping never
  does. Comparison is constant-time.
- In auto mode every fault (exception, encoder/gyro loss, distance or turn
  timeout, sensor read failure, obstacle after retries) stops the motors and
  pump and reports `ERROR` with the reason in `auto.message`.
- A manual drive command aborts a running sweep, so the operator always wins.

These are software backstops, not a substitute for a physical kill switch.

## Setup

Target: Raspberry Pi OS Bookworm or newer. Clone the project to
`/home/rover_dev/project` (the systemd units hard-code that path) and run the
installer **as the normal user, not with sudo**:

```bash
cd /home/rover_dev/project
bash deploy/install.sh                 # packages, venv, groups, units
WIFI_COUNTRY=NG bash deploy/install.sh # also set the Wi-Fi country
```

The installer is idempotent. It enables `rover-bt.service` but intentionally
does **not** enable `rover.service`; start the app from the BLE dashboard
(`START`), with `sudo systemctl start rover`, or run it by hand.

Boot configuration (check after the first run; a reboot is needed):

- Enable I²C for the MPU6050 and compass.
- Add `dtoverlay=dht11,gpiopin=21` to `/boot/firmware/config.txt`.
- Disable the serial console so the GPS can own `/dev/serial0`.

Camera (Bookworm or newer, with `rpicam-apps` installed):

```bash
bash deploy/setup_webrtc.sh
```

This downloads MediaMTX into `bin/` and installs `mediamtx.service`. Then open:

```
http://<pi-lan-ip>:5000          dashboard
http://<pi-lan-ip>:8889/rover    MediaMTX's built-in player
```

For development without the Pi:

```bash
python main.py    # http://localhost:5000
```

Hardware helpers fail soft when `pigpiod`, the sensors or the serial port are
absent, but the GPIO imports still need `gpiozero`/`pigpio` installed.

## Configuration

All settings are environment variables; unset values use the defaults below.
For the systemd service, add `Environment=NAME=value` lines to a local copy of
`deploy/rover.service`.

| Variable | Default | Purpose |
|---|---|---|
| `ROVER_PIN` | empty (no PIN) | Require a PIN for drive/probe/pump/camera |
| `WS_HEARTBEAT_TIMEOUT_SEC` | `10` | Drop a silent WebSocket after this long |
| `GPS_ENABLED` | `1` | Set `0` to skip the GPS entirely |
| `GPS_PORT` | `/dev/serial0` | Serial device |
| `GPS_BAUD` | `9600` | Serial baud rate |
| `GPS_MIN_SATS` | `6` | Minimum satellites before GPS fusion |
| `GPS_FUSION_ALPHA` | `0.08` | GPS position blend weight per update |
| `GPS_HEADING_BETA` | `0.04` | GPS course blend weight |
| `GPS_HEADING_MIN_SPEED_KMH` | `1.5` | Minimum speed for a usable course |
| `GPS_MAX_JUMP_M` | `12` | Reject position jumps larger than this |
| `COMPASS_ENABLED` | `1` | Set `0` to force the compass off |
| `COMPASS_FUSION_ALPHA` | `0.08` | Drift-correction weight |
| `COMPASS_SPIKE_DEG` | `15` | Single-sample jump treated as a glitch |
| `COMPASS_SPIKE_CONFIRM` | `3` | Readings needed to accept a spike |
| `IMU_STILL_DPS` | `1.0` | Gyro deadband that counts as parked |
| `IMU_STILL_TICKS` | `10` | Consecutive still ticks before gating |
| `IMU_BIAS_ALPHA` | `0.01` | Bias re-learning rate while parked |
| `SOIL_SENSOR_WET_WHEN_PRESSED` | `1` | Set `0` for inverted soil modules |
| `AUTO_STOP_ON_DISCONNECT` | `1` | Stop a sweep when the dashboard disconnects; `0` for unattended runs |
| `ROVER_AREAS_PATH` | `work_areas.json` | Where the map origin is stored |

`MALLOC_ARENA_MAX=2` is set in `rover.service` to keep the process small on a
512 MB board. The commented `MemoryMax`/`OOMScoreAdjust` lines are an optional
backstop; `deploy/setup_memory_hygiene.sh` and `deploy/setup_zram.sh` reduce
journal and swap pressure.

## Control API

### WebSocket — `/ws`

JSON messages. Protected commands also need `"pin"`.

| `type` | Fields | Effect |
|---|---|---|
| `motor` | `speed` 0–100, `dir` `forward`/`reverse`/`left`/`right`/`stop` | Drive |
| `sensor_deploy` | `action` `deploy`/`retract`/`toggle` | Probe arm |
| `pump` | `action` `on`/`off`/`toggle`/`status` | Water pump |
| `camera_tilt` | `direction` `left`/`right`/`center` | Camera pan |
| `auto_mode` | `action` `start`/`stop`/`toggle`/`status`, optional `params` object | Autonomous sweep (start needs PIN) |
| `ultrasonic` | `action` `enable`/`disable`/`toggle`/`status` | Ultrasonic power |
| `get_status` | — | Ask for telemetry |
| `ping` | — | Returns `pong` |

Responses carry a fresh `telemetry` payload, which is also broadcast to
connected clients.

### HTTP

| Method | Path | Auth | Purpose |
|---|---|---|---|
| GET | `/` | — | Dashboard |
| GET | `/motor?speed=&dir=` | PIN | Drive |
| GET | `/sensor_readings` | — | Full telemetry snapshot |
| GET/POST | `/ultrasonic?action=` | — | Ultrasonic enable/disable |
| GET/POST | `/deploy_sensor?action=` | PIN | Probe arm |
| GET/POST | `/water_pump?action=` | PIN | Water pump |
| GET | `/camera_tilt?direction=` | PIN | Camera pan |
| GET/POST | `/auto_mode?action=` | PIN to start | Sweep: `start`/`stop`/`toggle`/`status`; parameters in a JSON body |
| POST | `/api/navigation/origin` | — | `{"latitude", "longitude", "source"}` |
| POST | `/api/navigation/origin/gps` | — | Use the current GPS fix |
| POST | `/api/imu/calibrate` | — | Calibrate the MPU6050 (rover stopped) |
| GET | `/ws` | — | WebSocket upgrade |

When a PIN is required, send it as the `X-Rover-Pin` header or the `pin` query
parameter. The dashboard shows the PIN field automatically.

Starting a sweep:

```bash
curl -X POST http://<pi>:5000/auto_mode \
  -H 'Content-Type: application/json' \
  -d '{"action": "start", "params": {"length_m": 6, "width_m": 3,
       "row_spacing_m": 0.5, "turn_side": "right", "water_interval_m": 2,
       "pump_run_seconds": 3, "drive_speed": 0.85, "turn_speed": 0.9}}'

curl -X POST http://<pi>:5000/auto_mode -d '{"action": "stop"}' -H 'Content-Type: application/json'
```

## Bluetooth LE provisioning

`bt.py` advertises as **Smart-Rover** and is started by `rover-bt.service`
(root, because BlueZ D-Bus and `nmcli` need it). Service UUID
`12345678-1234-5678-1234-56789abcdef0` with these characteristics:

| UUID suffix | Access | Purpose |
|---|---|---|
| `…def1` | write | Wi-Fi credentials as `SSID:password` |
| `…def2` | write | App command |
| `…def3` | read | IP address (`hostname -I`) |
| `…def4` | read | Git update status |

App commands: `START`, `STOP`, `RESTART_ROVER`, `RESTART_MEDIAMTX`, `UPDATE`
(`git pull --ff-only` as `rover_dev`) and `COMMIT` (`git add -A` + commit with
a fixed message). Slow work runs off the BLE event loop so the connection is
not dropped.

The provisioning page is the root `index.html`. Web Bluetooth needs Chrome or
Edge and a secure context, so serve it over localhost or HTTPS:

```bash
cd /home/rover_dev/project
python -m http.server 8000     # then open http://localhost:8000/index.html
```

`bt.py` hard-codes `/home/rover_dev/project`; edit `PROJECT_DIR`/`APP_USER` if
the checkout lives elsewhere.

## Calibration

- **Wheels and encoders** — with the rover on a stand, run
  `python calibrate_wheels.py`. It drives one wheel at a time, reports signed
  tick counts and measures one output revolution. Put the measured value in
  `TICKS_PER_WHEEL_REV`, and the wheel diameter/track in
  `helpers/encoder_controller.py`. Auto mode derives every row length and step
  from those two constants (`auto_controller.METRES_PER_TICK`), so calibrate
  before trusting a sweep.
- **Motor polarity** — if the rover moves backwards when told to go forward,
  set `MOTOR_REVERSED = True` in `helpers/motor_controls.py` instead of
  rewiring.
- **IMU** — park the rover on a stable surface and use *Calibrate IMU* in the
  dashboard (or `POST /api/imu/calibrate`).
- **Compass** — no mounting calibration needed; the module learns its rotation
  sense on the first real turn. It fails soft if absent.
- **Soil polarity** — if wet and dry are swapped, set
  `SOIL_SENSOR_WET_WHEN_PRESSED=0`.

## Tests

The fusion and auto-mode planning tests are hardware-free and run on any
machine:

```bash
python test_fusion_logic.py
python test_auto_plan.py
```

On Windows, run it with `PYTHONIOENCODING=utf-8`; otherwise the compass
module's Unicode warning fails on the default `cp1252` console when the
hardware dependencies are absent.

`test_motor.py` is **not** a test — it drives both motors forward in a loop
until interrupted. Only run it with the rover lifted off the ground.

## Troubleshooting

| Symptom | Check |
|---|---|
| `pigpio daemon not running` | `sudo systemctl enable --now pigpiod` |
| No Bluetooth adapter / BLE not visible | `rfkill list`, `systemctl status bluetooth rover-bt` |
| Motors spin full speed regardless of slider | ENA/ENB jumper caps still fitted on the L298N |
| Rover pulls to one side / odometry drifts | Run `calibrate_wheels.py`; check encoder A/B wiring |
| Camera tile blank | `systemctl status mediamtx`; Bookworm or newer; ports 8889/tcp and 8189/udp reachable |
| GPS never gets a fix | Outdoors, antenna connected, serial console disabled, `GPS_ENABLED=1` |
| Dashboard shows `PIN required` | Enter the PIN in the header; it must match `ROVER_PIN` |
| Rover stops after a few seconds of driving | Dashboard pings are not reaching the Pi; check Wi-Fi and the 10 s heartbeat |
| HTTP works but WebSocket does not | A proxy is likely stripping the upgrade; connect directly to port 5000 |
| Auto mode refuses to start | Read `auto.error`: the MPU6050 (I2C) or the wheel encoders (`pigpiod`) are missing |
| Sweep stops mid-row with a timeout | `auto.state`/`auto.message` in telemetry; check `pigpiod`, wheel slip and `METRES_PER_TICK` |

Logs:

```bash
journalctl -u rover -f        # app
journalctl -u rover-bt -f     # BLE provisioning
journalctl -u mediamtx -f     # camera
```
