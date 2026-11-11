# Camera: Pi camera → H.264 → WebRTC

Live view path:

```
Pi camera ──libcamera──▶ BCM2835 hardware H.264 ──RTP/SRTP──▶ browser <video>
                          (MediaMTX rpiCamera source)         (WHEP)
```

MediaMTX owns the camera and does the encoding; the Flask app no longer forks
`rpicam-vid` at all. There is no M-JPEG path: WebRTC is the only way the
dashboard receives video.

## Install (on the Pi)

```bash
cd /home/rover_dev/project
bash deploy/setup_webrtc.sh          # downloads MediaMTX, installs + starts the unit
```

Requires Raspberry Pi OS **Bookworm or newer** (MediaMTX's `rpiCamera` source
uses libcamera) and `rpicam-vid` (`sudo apt install -y rpicam-apps`).

Check it:

```bash
systemctl status mediamtx
journalctl -u mediamtx -f
curl -s http://localhost:8889/rover | head -n1     # web player page
```

## Ports

| Port | Proto | Purpose |
|------|-------|---------|
| 8889 | TCP | WHEP signalling (HTTP) — the dashboard POSTs an SDP offer here |
| 8189 | UDP | WebRTC media (SRTP) |

If the LAN blocks UDP between the browser and the Pi, uncomment
`webrtcLocalTCPAddress: :8189` in `deploy/mediamtx.yml` to add an ICE/TCP
fallback (works, but adds a little latency).

If ICE cannot discover the Pi (multi-interface or Docker), set
`webrtcAdditionalHosts: [<pi-lan-ip>]`.

## Dashboard behaviour

`static/index.js` opens a WebRTC session to `http://<pi>:8889/rover/whep` on
page load and retries every 5 s if the connection drops or produces no video.
If it cannot connect the camera tile stays blank — there is no M-JPEG fallback.
Problems are logged to the browser console prefixed `📹`. A healthy stream logs
`WebRTC H.264 stream live.`

Camera only runs while someone watches: `sourceOnDemand: true` with
`sourceOnDemandCloseAfter: 15s`, so the encoder is idle when the dashboard is
closed.

## Tuning

In `deploy/mediamtx.yml`, per path:

- `rpiCameraWidth` / `rpiCameraHeight` — 320×240
- `rpiCameraFPS` — 8
- `rpiCameraBitrate` — 400 kbit/s
- `rpiCameraIDRPeriod` — keyframe interval in frames (8 = 1 s at 8 fps, fast joins)

These are deliberately low for the Zero W's single core; raise them only after
checking `top` while streaming.

After editing, `sudo systemctl restart mediamtx`.

## Verify without a browser

```bash
top -b -n1 | head -n12    # mediamtx CPU while streaming
```

RTSP is disabled in `deploy/mediamtx.yml`. To use `ffplay -rtsp_transport tcp
rtsp://127.0.0.1:8554/rover`, set `rtsp: true` and restart `mediamtx` first.
