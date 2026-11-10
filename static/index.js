const slider = document.getElementById("speedSlider");
const statusTxt = document.getElementById("statusTxt");
const targetVal = document.getElementById("targetVal");
const forwardBtn = document.getElementById("forwardBtn");
const reverseBtn = document.getElementById("reverseBtn");
const rightBtn = document.getElementById("rightBtn");
const leftBtn = document.getElementById("leftBtn");
const stopBtn = document.getElementById("stopBtn");
const sensorReadings = document.getElementById("sensorReadings");

const valTemp = document.getElementById("valTemp");
const valHumidity = document.getElementById("valHumidity");
const valSoil = document.getElementById("valSoil");
const valDist = document.getElementById("valDist");
const wsStatusTxt = document.getElementById("wsStatusTxt");
const connectionDot = document.getElementById("connectionDot");
const toast = document.getElementById("toast");
const throttleOutput = document.getElementById("throttleOutput");
const roverMap = document.getElementById("roverMap");
const poseText = document.getElementById("poseText");
const mapCoordinateText = document.getElementById("mapCoordinateText");
const gpsText = document.getElementById("gpsText");
const videoStream = document.getElementById("videoStream");
const valEncLeft = document.getElementById("valEncLeft");
const valEncRight = document.getElementById("valEncRight");
const compassCard = document.getElementById("compassCard");
const headingDigits = document.getElementById("headingDigits");
const pinInput = document.getElementById("roverPinInput");

// Build the compass card ticks once (every 10°, majors on the cardinals).
(function buildCompassTicks() {
  const group = document.getElementById("compassTicks");
  if (!group) return;
  const svgNS = "http://www.w3.org/2000/svg";
  for (let deg = 0; deg < 360; deg += 10) {
    const major = deg % 90 === 0;
    const path = document.createElementNS(svgNS, "path");
    path.setAttribute("d", `M100 ${100 - 94} V ${100 - (major ? 78 : 86)}`);
    path.setAttribute("transform", `rotate(${deg} 100 100)`);
    path.setAttribute("class", major ? "cc-tick major" : "cc-tick");
    group.appendChild(path);
  }
})();

// Dynamic base URLs
const baseURL = window.location.origin;
const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
const wsURL = `${wsProtocol}//${window.location.host}/ws`;

// Command PIN: the field appears in the header only when the server enforces
// one (telemetry pin_required), and the value persists locally.
let roverPin = localStorage.getItem("roverPin") || "";
if (pinInput) {
  pinInput.value = roverPin;
  pinInput.addEventListener("input", () => {
    roverPin = pinInput.value.trim();
    localStorage.setItem("roverPin", roverPin);
  });
}

function pinHeaders() {
  return roverPin ? { "X-Rover-Pin": roverPin } : {};
}

function roverFetch(url, opts = {}) {
  // Adds the PIN header and turns a 401 into a friendly, handled response
  // instead of a silently dead button (handlers surface data.error as a toast).
  return fetch(url, { ...opts, headers: { ...pinHeaders(), ...(opts.headers || {}) } }).then((res) => {
    if (res.status === 401) {
      if (pinInput) pinInput.focus();
      return {
        ok: false,
        status: 401,
        json: () => Promise.resolve({ success: false, error: "PIN required — enter it in the header" }),
      };
    }
    return res;
  });
}

let targetSpeed = 50;
let activeDirection = null; // Tracks active motor direction
const mapOriginDefault = [6.5244, 3.3792];
let mapOrigin = mapOriginDefault.slice();
let lastPose = null;
let leafletMap = null;
let roverMarker = null;
let trailLayer = null;
let lastGps = null;
let mapViewBootstrapped = false;
const trailPoints = [];
const EARTH_RADIUS_M = 6378137;

// WebSocket state
let ws = null;
let wsConnected = false;
let reconnectTimer = null;
let heartbeatInterval = null;
let httpPollInterval = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_DELAY_MS = 5000;

if (roverMap && window.L) {
  leafletMap = L.map(roverMap, { zoomControl: true, scrollWheelZoom: true, wheelPxPerZoomLevel: 100 }).setView(mapOrigin, 18);
  L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  }).addTo(leafletMap);
  trailLayer = L.polyline([], { color: "#a78bfa", weight: 3, opacity: 0.8 }).addTo(leafletMap);
  roverMarker = L.marker(mapOrigin, { icon: L.divIcon({ className: "rover-map-marker", html: "<span>▲</span>", iconSize: [28, 28], iconAnchor: [14, 14] }) }).addTo(leafletMap);
  leafletMap.on("click", (event) => {
    const lat = event.latlng.lat;
    const lon = event.latlng.lng;
    if (mapCoordinateText) mapCoordinateText.innerText = `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
  });
  leafletMap.on("mousemove", (event) => {
    if (mapCoordinateText) mapCoordinateText.innerText = `${event.latlng.lat.toFixed(5)}, ${event.latlng.lng.toFixed(5)}`;
  });
  window.addEventListener("resize", () => {
    if (leafletMap) leafletMap.invalidateSize();
  });
  const reflowMap = () => { if (leafletMap) leafletMap.invalidateSize(); };
  setTimeout(reflowMap, 100);
  setTimeout(reflowMap, 500);
  if (window.ResizeObserver && roverMap) {
    new ResizeObserver(reflowMap).observe(roverMap);
  }
}

// ----------------- Camera: WebRTC (H.264) only -----------------
// MediaMTX on the Pi owns the camera, hardware-encodes H.264 and publishes it
// over WebRTC (WHEP on port 8889). The <video> is the only feed: there is no
// MJPEG path and no MJPEG fallback.

const CAMERA_PATH = "rover";
const WEBRTC_WHEP_URL = `http://${location.hostname}:8889/${CAMERA_PATH}/whep`;
const WEBRTC_RETRY_MS = 5000;
let peerConnection = null;
let webrtcRetryTimer = null;
let webrtcVideoTimer = null;
// The feed is a deliberate choice, not a page-load side effect: switching it
// off closes the WHEP session, and MediaMTX's sourceOnDemand then shuts the
// Pi camera and hardware encoder down ~15 s later.
let cameraFeedOn = true;
// WHEP hands back a session URL in the Location header; DELETE ends the session
// so MediaMTX stops the source now instead of noticing the reader is gone.
let whepSessionUrl = null;

function setCameraUi() {
  const button = document.getElementById("cameraToggleBtn");
  const tag = document.getElementById("cameraLiveTag");
  const tagTxt = document.getElementById("cameraStateTxt");
  const off = document.getElementById("cameraOff");
  if (button) {
    button.textContent = cameraFeedOn ? "Stop" : "Start";
    button.setAttribute("aria-pressed", cameraFeedOn ? "true" : "false");
  }
  if (tag) tag.classList.toggle("off", !cameraFeedOn);
  if (tagTxt) tagTxt.textContent = cameraFeedOn ? "LIVE" : "OFF";
  if (off) off.hidden = cameraFeedOn;
}

function closeWebRTC() {
  clearTimeout(webrtcVideoTimer);
  clearTimeout(webrtcRetryTimer);
  if (whepSessionUrl) {
    const url = whepSessionUrl;
    whepSessionUrl = null;
    // Best effort: if DELETE is refused the closed peer connection still tears
    // the session down, just less promptly.
    fetch(url, { method: "DELETE" }).catch(() => {});
  }
  if (peerConnection) {
    try { peerConnection.close(); } catch (_) {}
    peerConnection = null;
  }
  if (videoStream) {
    videoStream.srcObject = null;
    if (videoStream.pause) videoStream.pause();
  }
}

function startCameraFeed(reason) {
  cameraFeedOn = true;
  setCameraUi();
  return connectWebRTC(reason || "user start");
}

function stopCameraFeed(reason) {
  cameraFeedOn = false;
  closeWebRTC();
  setCameraUi();
  console.log(`📹 Camera feed stopped${reason ? ` (${reason})` : ""}.`);
}

function toggleCameraFeed() {
  if (cameraFeedOn) {
    stopCameraFeed("user");
    showToast("Camera feed stopped");
  } else {
    showToast("Opening camera feed…");
    startCameraFeed("user start");
  }
}

function retryWebRTC(reason) {
  closeWebRTC();
  if (!cameraFeedOn) return;
  console.warn(`📹 WebRTC: ${reason}. Retrying in ${WEBRTC_RETRY_MS / 1000}s.`);
  webrtcRetryTimer = setTimeout(() => connectWebRTC(reason), WEBRTC_RETRY_MS);
}

async function connectWebRTC(reason) {
  if (!cameraFeedOn) return false;
  if (!videoStream || !window.RTCPeerConnection) {
    console.warn("📹 WebRTC is not supported by this browser.");
    return false;
  }
  closeWebRTC();
  const connection = new RTCPeerConnection();
  peerConnection = connection;
  connection.addTransceiver("video", { direction: "recvonly" });

  connection.ontrack = (event) => {
    if (connection !== peerConnection || !cameraFeedOn) return;
    clearTimeout(webrtcVideoTimer);
    videoStream.srcObject = event.streams[0];
    const play = videoStream.play();
    if (play && play.catch) play.catch(() => {});
    console.log("📹 WebRTC H.264 stream live.");
  };

  connection.onconnectionstatechange = () => {
    if (connection !== peerConnection) return;
    const state = connection.connectionState;
    if (state === "failed" || state === "disconnected") retryWebRTC(`connection ${state}`);
  };

  try {
    const offer = await connection.createOffer();
    await connection.setLocalDescription(offer);
    const response = await fetch(WEBRTC_WHEP_URL, {
      method: "POST",
      headers: { "Content-Type": "application/sdp" },
      body: offer.sdp,
    });
    if (!response.ok) throw new Error(`WHEP HTTP ${response.status}`);
    const location = response.headers.get("Location");
    whepSessionUrl = location ? new URL(location, WEBRTC_WHEP_URL).toString() : null;
    const answer = await response.text();
    if (connection !== peerConnection) return false;
    await connection.setRemoteDescription({ type: "answer", sdp: answer });
    console.log(`📹 WebRTC negotiated (${reason}).`);
    webrtcVideoTimer = setTimeout(() => {
      if (connection === peerConnection && !videoStream.srcObject) retryWebRTC("no video received");
    }, 10000);
    return true;
  } catch (err) {
    if (connection === peerConnection) retryWebRTC(err.message || "signalling failed");
    return false;
  }
}

// ----------------- Connection Badge & State -----------------

function updateConnectionBadge(state, text) {
  if (wsStatusTxt) {
    wsStatusTxt.innerText = text
      || (state === "connected" ? "ONLINE" : state === "connecting" ? "CONNECTING" : "OFFLINE");
    wsStatusTxt.className = `ws-state ${state}`;
  }
  if (connectionDot) {
    connectionDot.className = state === "connected" ? "status-dot online" : "status-dot";
  }
}

// ----------------- WebSocket Engine & Reconnection -----------------

function initWebSocket() {
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
    return;
  }

  updateConnectionBadge("connecting", reconnectAttempts > 0 ? `RETRY #${reconnectAttempts} 🔄` : "CONNECTING ⚡");
  console.log(`[WS] Attempting connection to ${wsURL}...`);

  try {
    ws = new WebSocket(wsURL);
  } catch (err) {
    console.error("[WS] Initialization error:", err);
    scheduleReconnect();
    return;
  }

  ws.onopen = () => {
    console.log("⚡ [WS] Connected to rover successfully.");
    wsConnected = true;
    reconnectAttempts = 0;
    updateConnectionBadge("connected", "ONLINE ⚡");

    ensureHttpPolling(false);
    sendWSCommand({ type: "get_status" });

    clearInterval(heartbeatInterval);
    heartbeatInterval = setInterval(() => {
      if (ws && ws.readyState === WebSocket.OPEN) {
        sendWSCommand({ type: "ping" });
      }
    }, 2000);
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);

      if (msg.type === "telemetry" && msg.data) {
        renderTelemetry(msg.data);
      } else if (msg.type === "connection_ack" && msg.data) {
        console.log("⚡ [WS] Hardware state synchronized after connect/reconnect.");
        renderTelemetry(msg.data);
      } else if (msg.type === "response") {
        if (msg.status && statusTxt) {
          statusTxt.innerText = msg.status;
        }
        if (msg.telemetry) {
          renderTelemetry(msg.telemetry);
        }
        notifyFromResponse(msg);
      } else if (msg.type === "error" && msg.message) {
        showToast(msg.message);
      }
    } catch (err) {
      console.warn("[WS] Error parsing incoming message:", err, event.data);
    }
  };

  ws.onerror = (err) => {
    console.warn("⚠️ [WS] Connection error:", err);
  };

  ws.onclose = (event) => {
    console.warn(`🔌 [WS] Disconnected (code: ${event.code}). Manual drive stopped.`);
    handleDisconnect();
  };
}

function handleDisconnect() {
  wsConnected = false;
  clearInterval(heartbeatInterval);

  activeDirection = null;
  if (statusTxt) statusTxt.innerText = "STOPPED (SAFE)";

  updateConnectionBadge("connecting", "RECONNECTING 🔄");
  ensureHttpPolling(true);
  scheduleReconnect();
}

function scheduleReconnect() {
  clearTimeout(reconnectTimer);
  reconnectAttempts++;
  const delay = Math.min(1000 * Math.pow(1.4, reconnectAttempts - 1), MAX_RECONNECT_DELAY_MS);
  console.log(`[WS] Scheduling reconnect attempt #${reconnectAttempts} in ${Math.round(delay)}ms...`);
  reconnectTimer = setTimeout(initWebSocket, delay);
}

function sendWSCommand(payload) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    try {
      if (roverPin) payload.pin = roverPin;
      ws.send(JSON.stringify(payload));
      return true;
    } catch (e) {
      console.error("[WS] Send error:", e);
      return false;
    }
  }
  return false;
}

// ----------------- HTTP Polling Fallback -----------------

function ensureHttpPolling(enable) {
  if (enable) {
    if (!httpPollInterval) {
      console.log("📡 Enabling HTTP telemetry polling fallback...");
      httpPollInterval = setInterval(fetchSensorDataHttp, 2500);
      fetchSensorDataHttp();
    }
  } else {
    if (httpPollInterval) {
      console.log("📡 Disabling HTTP polling: WebSocket is active.");
      clearInterval(httpPollInterval);
      httpPollInterval = null;
    }
  }
}

function fetchSensorDataHttp() {
  fetch(`${baseURL}/sensor_readings`)
    .then((res) => res.json())
    .then((data) => {
      if (data.success) {
        renderTelemetry(data);
      }
    })
    .catch((err) => console.warn("HTTP Telemetry fetch error:", err));
}

// ----------------- Compass instrument -----------------
// Aviation pattern: the heading arrow stays fixed pointing up (the
// vehicle's nose) while the card rotates behind it under the lubber line.
// Pose/IMU heading is math style (0° = east, CCW); the compass shows
// 0° = north, clockwise.

let compassRotation = 0;
let compassPrev = null;

function compassDeg(mathDeg) {
  return ((90 - mathDeg) % 360 + 360) % 360;
}

function updateCompass(data) {
  let heading = null;
  if (data.pose && Number.isFinite(data.pose.heading_deg)) {
    heading = compassDeg(data.pose.heading_deg);
  } else if (data.imu && data.imu.available && Number.isFinite(data.imu.heading_deg)) {
    heading = compassDeg(data.imu.heading_deg); // relative gyro fallback
  }
  if (heading === null) return;

  if (compassPrev === null) {
    // First fix: start from due north so the card sweeps in once.
    compassPrev = heading;
    compassRotation = heading;
  } else {
    // Always take the short way round so 359° → 01° nudges instead of spinning.
    const delta = ((heading - compassPrev + 180) % 360 + 360) % 360 - 180;
    compassRotation += delta;
    compassPrev = heading;
  }
  if (compassCard) compassCard.style.transform = `rotate(${-compassRotation}deg)`;
  if (headingDigits) {
    headingDigits.innerText = `${String(Math.round(heading) % 360).padStart(3, "0")}\u00b0`;
  }
}

// ----------------- Telemetry Renderer -----------------

function renderTelemetry(data) {
  if (!data) return;

  if (valTemp) valTemp.innerText = data.temperature !== undefined ? `${data.temperature}°C` : '--';
  if (valHumidity) valHumidity.innerText = data.humidity !== undefined ? `${data.humidity}%` : '--';
  if (valSoil) {
    valSoil.innerText = data.soil || "Unknown";
    const diag = data.soil_diagnostics || {};
    valSoil.title = [
      diag.pressed === null || diag.pressed === undefined
        ? "GPIO input: unavailable"
        : `GPIO input: ${diag.pressed ? "LOW/pressed" : "HIGH/released"}`,
      diag.error || "",
    ].filter(Boolean).join(" · ");
  }
  if (valDist) valDist.innerText = data.distance_cm !== undefined ? `${data.distance_cm} cm` : '--';
  if (valEncLeft) valEncLeft.innerText = data.encoder_left !== undefined ? data.encoder_left : '0';
  if (valEncRight) valEncRight.innerText = data.encoder_right !== undefined ? data.encoder_right : '0';

  if (pinInput && data.pin_required !== undefined) {
    pinInput.hidden = !data.pin_required;
  }

  if (sensorReadings) {
    sensorReadings.innerHTML = `
      <p>Temp: ${data.temperature}°C</p>
      <p>Humidity: ${data.humidity}%</p>
      <p>Soil: ${data.soil}</p>
      <p>Dist: ${data.distance_cm}cm</p>
    `;
  }

  if (data.pump_active !== undefined) updatePumpUI(data.pump_active);
  if (data.sensor_deployed !== undefined) updateSensorUI(data.sensor_deployed);

  if (data.motor_status !== undefined && statusTxt) {
    statusTxt.innerText = data.motor_status;
  }
  if (data.camera_angle !== undefined) {
    const angleElem = document.getElementById("current-angle");
    if (angleElem) angleElem.innerText = data.camera_angle;
  }
  
  if (data.gps) updateGpsReadout(data.gps);
  if (data.pose) renderRoverMap(data.pose);
  if (leafletMap && !mapViewBootstrapped) {
    if (data.gps && data.gps.fix && data.gps.lat != null) {
      leafletMap.setView([data.gps.lat, data.gps.lon], 18);
      mapViewBootstrapped = true;
    } else if (Array.isArray(data.map_origin)) {
      mapOrigin = data.map_origin;
      leafletMap.setView(mapOrigin, 18);
      roverMarker.setLatLng(mapOrigin);
      mapViewBootstrapped = true;
    }
  }

  if (data.imu) {
    const pitch = Number.isFinite(data.imu.pitch_deg) ? data.imu.pitch_deg : 0;
    const roll = Number.isFinite(data.imu.roll_deg) ? data.imu.roll_deg : 0;
    // Standalone attitude sprite: the horizon ball tilts with roll and
    // slides with pitch; the fixed aircraft glyph is the reference.
    const horizon = document.getElementById("imuHorizon");
    if (horizon) {
      const pitchPx = Math.max(-40, Math.min(40, pitch)) * 1.1;
      horizon.setAttribute("transform", `translate(0 ${pitchPx}) rotate(${-roll} 60 60)`);
    }
    const imuDigits = document.getElementById("imuDigits");
    if (imuDigits) {
      const fmt = (v) => `${v >= 0 ? "+" : "−"}${Math.abs(v).toFixed(1)}°`;
      imuDigits.innerText = `P ${fmt(pitch)} R ${fmt(roll)}`;
    }
  }
  updateCompass(data);
}

function updateGpsReadout(gps) {
  lastGps = gps || null;
  if (!gpsText) return;
  if (!gps || !gps.available) {
    gpsText.innerText = gps && gps.error ? `GPS off · ${gps.error}` : "GPS unavailable";
    gpsText.classList.remove("fix");
    return;
  }
  if (gps.fix && gps.lat != null) {
    gpsText.innerText = `GPS fix · ${gps.sats} sats · ${gps.lat.toFixed(5)}, ${gps.lon.toFixed(5)}`;
    gpsText.classList.add("fix");
  } else {
    gpsText.innerText = `GPS search · ${gps.sats || 0} sats`;
    gpsText.classList.remove("fix");
  }
}

function getRoverGeoPosition() {
  if (lastGps && lastGps.fix && lastGps.lat != null) {
    return [lastGps.lat, lastGps.lon];
  }
  if (roverMarker) {
    const ll = roverMarker.getLatLng();
    return [ll.lat, ll.lng];
  }
  if (lastPose) {
    return localToGeo(lastPose.x_m, lastPose.y_m);
  }
  return null;
}

function notifyFromResponse(msg) {
  if (!msg || msg.type !== "response") return;
  if (msg.success === false) {
    showToast(msg.message || msg.error || "Command failed");
    return;
  }
  const cmd = msg.command;
  if (cmd === "ping" || cmd === "get_status") return;
  if (cmd === "motor" && msg.status) {
    if (/STOPPED/i.test(msg.status)) showToast("Motors stopped");
    return;
  }
  if (cmd === "pump") {
    showToast(msg.pump_active ? "Water pump on" : "Water pump off");
  } else if (cmd === "sensor_deploy") {
    showToast(msg.is_deployed ? "Probe deployed" : "Probe retracted");
  } else if (cmd === "camera_tilt" && msg.current_angle != null) {
    showToast(`Camera pan ${msg.current_angle}°`);
  }
}

function geoToLocal(point, origin = mapOrigin) {
  const lat = point[0] * Math.PI / 180;
  const originLat = origin[0] * Math.PI / 180;
  return [
    EARTH_RADIUS_M * Math.cos(originLat) * (point[1] - origin[1]) * Math.PI / 180,
    EARTH_RADIUS_M * (point[0] - origin[0]) * Math.PI / 180,
  ];
}

function localToGeo(x, y, origin = mapOrigin) {
  return [origin[0] + (y / EARTH_RADIUS_M) * 180 / Math.PI,
    origin[1] + (x / (EARTH_RADIUS_M * Math.cos(origin[0] * Math.PI / 180))) * 180 / Math.PI];
}

function renderRoverMap(pose) {
  if (!leafletMap || !pose) return;
  lastPose = pose;
  const roverGeo = localToGeo(pose.x_m, pose.y_m);
  roverMarker.setLatLng(roverGeo);
  roverMarker.setIcon(L.divIcon({ className: "rover-map-marker", html: `<span style="transform:rotate(${90 - (pose.heading_deg || 0)}deg)">▲</span>`, iconSize: [28, 28], iconAnchor: [14, 14] }));
  trailPoints.push(roverGeo);
  if (trailPoints.length > 200) trailPoints.shift();
  trailLayer.setLatLngs(trailPoints);
  if (poseText) {
    let label = `${pose.x_m.toFixed(1)}m, ${pose.y_m.toFixed(1)}m · ${pose.heading_deg.toFixed(0)}°`;
    const fusion = pose.fusion;
    if (fusion && fusion.gps_position) label += " · GPS fused";
    else if (fusion && fusion.mode === "odometry") label += " · enc+IMU";
    poseText.innerText = label;
  }
}

function centerMapOnRover() {
  if (!leafletMap) return;
  const pos = getRoverGeoPosition();
  if (!pos) {
    showToast("Rover position not available yet");
    return;
  }
  leafletMap.setView(pos, Math.max(leafletMap.getZoom(), 18));
  showToast("Map centred on rover");
}

function updateSensorUI(isDeployed) {
  const sensorStatus = document.getElementById("sensorStatus");
  const sensorBtn = document.getElementById("sensorDeployBtn");
  if (sensorStatus && sensorBtn) {
    if (isDeployed) {
      sensorStatus.innerText = "DEPLOYED (-20°)";
      sensorBtn.classList.add("active");
    } else {
      sensorStatus.innerText = "RETRACTED (-90°)";
      sensorBtn.classList.remove("active");
    }
  }
}

function updatePumpUI(isActive) {
  const pumpStatus = document.getElementById("pumpStatus");
  const pumpBtn = document.getElementById("pumpToggleBtn");
  if (pumpStatus && pumpBtn) {
    if (isActive) {
      pumpStatus.innerText = "RUNNING 💧";
      pumpBtn.classList.add("active");
    } else {
      pumpStatus.innerText = "OFF ⏹️";
      pumpBtn.classList.remove("active");
    }
  }
}

// ----------------- Command Handlers (WS with HTTP Fallback) -----------------

function sendHardwareCommand(speed, direction) {
  const payload = { type: "motor", speed: speed, dir: direction };
  const sent = sendWSCommand(payload);

  if (!sent) {
    roverFetch(`${baseURL}/motor?speed=${speed}&dir=${direction}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success && statusTxt) statusTxt.innerText = data.status;
      })
      .catch((err) => console.error("Motor HTTP fallback error:", err));
  }
}

function toggleSensorDeploy(action = "toggle") {
  const sent = sendWSCommand({ type: "sensor_deploy", action: action });
  if (!sent) {
    roverFetch(`${baseURL}/deploy_sensor?action=${action}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success) {
          updateSensorUI(data.is_deployed);
          showToast(data.is_deployed ? "Probe deployed" : "Probe retracted");
        } else showToast(data.error || "Probe command failed");
      })
      .catch(() => showToast("Could not reach rover (probe)"));
  }
}

function toggleWaterPump(action = "toggle") {
  const sent = sendWSCommand({ type: "pump", action: action });
  if (!sent) {
    roverFetch(`${baseURL}/water_pump?action=${action}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success) {
          updatePumpUI(data.pump_active);
          showToast(data.pump_active ? "Water pump on" : "Water pump off");
        } else showToast(data.error || "Pump command failed");
      })
      .catch(() => showToast("Could not reach rover (pump)"));
  }
}

function tiltCamera(direction) {
  const sent = sendWSCommand({ type: "camera_tilt", direction: direction });
  if (!sent) {
    roverFetch(`${baseURL}/camera_tilt?direction=${direction}`)
      .then((res) => res.json())
      .then((data) => {
        const angleElem = document.getElementById("current-angle");
        if (angleElem && data.current_angle !== undefined) {
          angleElem.innerText = data.current_angle;
        }
        if (data.success) showToast(`Camera pan ${data.current_angle}°`);
        else showToast(data.error || "Camera tilt failed");
      })
      .catch(() => showToast("Could not reach rover (camera)"));
  }
}

// ----------------- Slider & Input Controls -----------------

if (slider) {
  slider.oninput = function () {
    targetSpeed = this.value;
    if (targetVal) targetVal.innerHTML = targetSpeed;
    if (throttleOutput) throttleOutput.innerText = `${targetSpeed}%`;
    if (activeDirection) {
      sendHardwareCommand(targetSpeed, activeDirection);
    }
  };
}

function openModal(id) {
  const modal = document.getElementById(id);
  if (modal) modal.classList.add("open");
}

function closeModal(id) {
  const modal = document.getElementById(id);
  if (modal) modal.classList.remove("open");
}

function showToast(message) {
  if (!toast) return;
  toast.innerText = message;
  toast.classList.add("show");
  clearTimeout(window.roverToastTimer);
  window.roverToastTimer = setTimeout(() => toast.classList.remove("show"), 2800);
}

function calibrateIMU() {
  showToast("Keep the rover still — calibrating…");
  fetch(`${baseURL}/api/imu/calibrate`, { method: "POST" })
    .then((res) => res.json().then((data) => ({ ok: res.ok, data })))
    .then(({ ok, data }) => {
      closeModal("imuModal");
      showToast(ok ? "MPU6050 calibrated" : (data.error || "Calibration failed"));
    })
    .catch(() => showToast("Unable to reach the rover"));
}

function setupButton(buttonElement, directionName) {
  if (!buttonElement) return;

  const startAction = (e) => {
    e.preventDefault();
    if (!activeDirection) {
      activeDirection = directionName;
      sendHardwareCommand(targetSpeed, directionName);
    }
  };

  const stopAction = (e) => {
    e.preventDefault();
    if (activeDirection === directionName) {
      activeDirection = null;
      sendHardwareCommand(0, "stop");
    }
  };

  buttonElement.addEventListener("mousedown", startAction);
  buttonElement.addEventListener("mouseup", stopAction);
  buttonElement.addEventListener("mouseleave", stopAction);

  buttonElement.addEventListener("touchstart", startAction);
  buttonElement.addEventListener("touchend", stopAction);
  buttonElement.addEventListener("touchcancel", stopAction);
}

setupButton(forwardBtn, "forward");
setupButton(reverseBtn, "reverse");
setupButton(rightBtn, "right");
setupButton(leftBtn, "left");

if (stopBtn) {
  stopBtn.addEventListener("click", (e) => {
    e.preventDefault();
    activeDirection = null;
    sendHardwareCommand(0, "stop");
  });
}

// Keyboard shortcuts for Desktop
document.addEventListener("keydown", (event) => {
  if (document.activeElement.tagName === "INPUT") return;

  const key = event.key.toLowerCase();

  if (key === "arrowup" || key === "w") {
    if (!activeDirection) {
      activeDirection = "forward";
      sendHardwareCommand(targetSpeed, "forward");
    }
  } else if (key === "arrowdown" || key === "s") {
    if (!activeDirection) {
      activeDirection = "reverse";
      sendHardwareCommand(targetSpeed, "reverse");
    }
  } else if (key === "arrowleft" || key === "a") {
    if (!activeDirection) {
      activeDirection = "left";
      sendHardwareCommand(targetSpeed, "left");
    }
  } else if (key === "arrowright" || key === "d") {
    if (!activeDirection) {
      activeDirection = "right";
      sendHardwareCommand(targetSpeed, "right");
    }
  } else if (key === " " || key === "spacebar") {
    activeDirection = null;
    sendHardwareCommand(0, "stop");
  } else if (key === "c") {
    tiltCamera("center");
  } else if (key === "e" || key === "m") {
    toggleSensorDeploy();
  } else if (key === "p") {
    toggleWaterPump();
  }
});

document.addEventListener("keyup", (event) => {
  if (document.activeElement.tagName === "INPUT") return;
  const key = event.key.toLowerCase();

  if (
    (key === "arrowup" || key === "w") && activeDirection === "forward" ||
    (key === "arrowdown" || key === "s") && activeDirection === "reverse" ||
    (key === "arrowleft" || key === "a") && activeDirection === "left" ||
    (key === "arrowright" || key === "d") && activeDirection === "right"
  ) {
    activeDirection = null;
    sendHardwareCommand(0, "stop");
  }
});

window.addEventListener("blur", () => {
  if (activeDirection) {
    activeDirection = null;
    sendHardwareCommand(0, "stop");
  }
});

window.addEventListener("beforeunload", () => {
  closeWebRTC();
  if (activeDirection) {
    sendHardwareCommand(0, "stop");
  }
  if (ws) {
    try {
      ws.close();
    } catch (_) {}
  }
});

// Camera boot: open the feed on page load, as before; the Stop button hands
// control to the user afterwards.
setCameraUi();
if (videoStream) connectWebRTC("page load");
// Closing the tab should release the Pi camera too, not wait for a timeout.
window.addEventListener("pagehide", closeWebRTC);

// Initialize WebSocket on page load
initWebSocket();

