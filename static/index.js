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
const modeTxt = document.getElementById("modeTxt");
const wsStatusTxt = document.getElementById("wsStatusTxt");
const connectionDot = document.getElementById("connectionDot");
const toast = document.getElementById("toast");
const missionPercent = document.getElementById("missionPercent");
const throttleOutput = document.getElementById("throttleOutput");
const areaSummary = document.getElementById("areaSummary");

// New Auto Config Elements
const autoSpeedSlider = document.getElementById("autoSpeedSlider");
const autoSpeedVal = document.getElementById("autoSpeedVal");
const autoProgressText = document.getElementById("autoProgressText");
const autoProgressFill = document.getElementById("autoProgressFill");
const areaNameInput = document.getElementById("areaNameInput");
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
let roverAutoActive = false;
const mapOriginDefault = [6.5244, 3.3792];
let mapOrigin = mapOriginDefault.slice();
let lastPose = null;
let leafletMap = null;
let roverMarker = null;
let workAreaLayer = null;
let areaPreviewLayer = null;
let trailLayer = null;
let loadedAreaId = null;
let loadedOrigin = false;
let lastGps = null;
let originSetForWork = false;
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
  workAreaLayer = L.polygon([], { color: "#38d9cf", weight: 2, fillColor: "#38d9cf", fillOpacity: 0.13 }).addTo(leafletMap);
  areaPreviewLayer = L.polygon([], { color: "#f2ba66", weight: 2, dashArray: "4 4", fillColor: "#f2ba66", fillOpacity: 0.1 }).addTo(leafletMap);
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
    console.warn(`🔌 [WS] Disconnected (code: ${event.code}). Manual drive stopped; an autonomous mission keeps running.`);
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
  if (valSoil) valSoil.innerText = data.soil || 'Dry';
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
  if (data.auto_active !== undefined) updateAutoUI(data.auto_active);
  
  if (data.motor_status !== undefined && statusTxt) {
    statusTxt.innerText = data.motor_status;
  }
  if (data.camera_angle !== undefined) {
    const angleElem = document.getElementById("current-angle");
    if (angleElem) angleElem.innerText = data.camera_angle;
  }
  
  if (data.auto_progress) {
    updateAutoProgressUI(data.auto_progress);
  }
  if (data.gps) updateGpsReadout(data.gps);
  if (data.pose) renderRoverMap(data.pose, data.active_area);
  if (data.active_area !== undefined) updateAreaSummary(data.active_area);
  if (leafletMap && !mapViewBootstrapped) {
    if (data.gps && data.gps.fix && data.gps.lat != null) {
      leafletMap.setView([data.gps.lat, data.gps.lon], 18);
      mapViewBootstrapped = true;
    } else if (Array.isArray(data.map_origin)) {
      mapOrigin = data.map_origin;
      leafletMap.setView(mapOrigin, 18);
      roverMarker.setLatLng(mapOrigin);
      loadedOrigin = true;
      originSetForWork = true;
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

function applyOriginAtRover(sourceLabel) {
  const selectedOrigin = getRoverGeoPosition();
  if (!selectedOrigin) {
    showToast("No rover position yet — wait for GPS or encoder data");
    return Promise.reject(new Error("no position"));
  }
  const previousOrigin = mapOrigin.slice();
  mapOrigin = selectedOrigin.slice();
  if (leafletMap) {
    leafletMap.setView(selectedOrigin, Math.max(leafletMap.getZoom(), 18));
  }
  if (mapCoordinateText) {
    mapCoordinateText.innerText = `${selectedOrigin[0].toFixed(5)}, ${selectedOrigin[1].toFixed(5)}`;
  }
  const body = {
    latitude: selectedOrigin[0],
    longitude: selectedOrigin[1],
    source: lastGps && lastGps.fix ? "gps" : "rover",
  };
  return fetch(`${baseURL}/api/navigation/origin`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  })
    .then((response) => response.json().then((data) => ({ ok: response.ok, data })))
    .then(({ ok, data }) => {
      if (!ok) throw new Error(data.error || "Could not set work origin");
      trailPoints.length = 0;
      if (trailLayer) trailLayer.setLatLngs([]);
      loadedOrigin = true;
      originSetForWork = true;
      lastPose = { x_m: 0, y_m: 0, heading_deg: lastPose ? lastPose.heading_deg : 0 };
      roverMarker.setLatLng(selectedOrigin);
      if (poseText) {
        poseText.innerText = `0.0m, 0.0m · ${(lastPose.heading_deg || 0).toFixed(0)}°`;
      }
      showToast(sourceLabel || "Map centred on rover — origin set to 0, 0 m");
    })
    .catch((error) => {
      mapOrigin = previousOrigin;
      showToast(error.message || "Could not set origin");
      throw error;
    });
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
  } else if (cmd === "auto_mode") {
    showToast(msg.auto_active ? "Autonomous mode started" : "Autonomous mode stopped");
  } else if (cmd === "auto_speed" && msg.auto_speed_pct != null) {
    showToast(`Auto speed set to ${msg.auto_speed_pct}%`);
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

function renderRoverMap(pose, area) {
  if (!leafletMap || !pose) return;
  lastPose = pose;
  if (area) {
    window.activeWorkArea = area;
    if (loadedAreaId !== area.id) {
      if (Number.isFinite(area.origin_lat) && Number.isFinite(area.origin_lon)) {
        mapOrigin = [area.origin_lat, area.origin_lon];
        loadedOrigin = true;
        originSetForWork = true;
      }
      const areaPoints = Array.isArray(area.map_polygon) ? area.map_polygon : [];
      workAreaLayer.setLatLngs(areaPoints);
      if (areaPoints.length >= 3) {
        leafletMap.fitBounds(workAreaLayer.getBounds(), { padding: [24, 24], maxZoom: 18 });
      } else if (Number.isFinite(area.origin_lat) && Number.isFinite(area.origin_lon)) {
        leafletMap.setView(mapOrigin, 18);
      }
      loadedAreaId = area.id;
    }
  }
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

function setRoverStart() {
  if (!leafletMap) return;
  applyOriginAtRover("Work map centred on rover — local position reset to 0, 0 m");
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

// ----------------- Work area: size only -----------------
// The mission sweeps one explicit area, defined by how much ground it covers
// east-west (breadth) and north-south (length).  It is centred on the rover
// where it stands when the area is set, so no coordinates are needed.

function areaNumber(id, fallback) {
  const input = document.getElementById(id);
  const value = input ? parseFloat(input.value) : NaN;
  return Number.isFinite(value) ? value : fallback;
}

function areaForm() {
  return {
    width: Math.min(100, Math.max(0.5, areaNumber("areaWidth", 6))),
    length: Math.min(100, Math.max(0.5, areaNumber("areaLength", 6))),
    swath: Math.min(5, Math.max(0.1, areaNumber("areaSwath", 0.5))),
    name: ((document.getElementById("areaNameInput") || {}).value || "Work area").trim() || "Work area",
  };
}

// Same rectangle the server builds: centred on the rover, axis-aligned local
// metres.  Nothing is drawn until a map origin exists to place it on the map.
function areaCornersGeo(form) {
  if (!originSetForWork && !loadedOrigin) return [];
  const pose = lastPose || { x_m: 0, y_m: 0 };
  return [
    localToGeo(pose.x_m - form.width / 2, pose.y_m - form.length / 2),
    localToGeo(pose.x_m + form.width / 2, pose.y_m - form.length / 2),
    localToGeo(pose.x_m + form.width / 2, pose.y_m + form.length / 2),
    localToGeo(pose.x_m - form.width / 2, pose.y_m + form.length / 2),
  ];
}

// Mirrors helpers/auto_plan.plan_rows: rows every swath*0.9 from the near edge,
// plus a last pass when the spacing would leave the far edge dry.
function planPassCount(form) {
  const spacing = form.swath * 0.9;            // 10% overlap, as the planner does
  const across = Math.min(form.width, form.length);
  let rows = Math.floor(across / spacing + 1e-9) + 1;
  if ((rows - 1) * spacing < across - form.swath / 2) rows += 1;
  return Math.max(1, rows);
}

function previewWorkArea() {
  const form = areaForm();
  const corners = areaCornersGeo(form);
  if (areaPreviewLayer) areaPreviewLayer.setLatLngs(corners.length === 4 ? corners : []);
  const note = document.getElementById("areaPlanNote");
  if (!note) return;
  const passes = planPassCount(form);
  const distance = passes * Math.max(form.width, form.length);
  note.innerText = `${form.width.toFixed(1)} × ${form.length.toFixed(1)} m at ${form.swath.toFixed(2)} m per pass → ${passes} passes, about ${distance.toFixed(0)} m of driving, centred on the rover.`;
}

function updateAreaSummary(area) {
  if (!areaSummary) return;
  if (!area) {
    areaSummary.innerText = "No work area set — open Work area…";
    return;
  }
  const size = Number.isFinite(area.width_m) && Number.isFinite(area.height_m)
    ? ` · ${area.width_m.toFixed(1)} × ${area.height_m.toFixed(1)} m`
    : "";
  areaSummary.innerText = `${area.name || "Work area"}${size}`;
}

function openWorkArea() {
  const area = window.activeWorkArea;
  if (area) {
    const widthInput = document.getElementById("areaWidth");
    const lengthInput = document.getElementById("areaLength");
    const swathInput = document.getElementById("areaSwath");
    const nameInput = document.getElementById("areaNameInput");
    if (widthInput && Number.isFinite(area.width_m)) widthInput.value = area.width_m;
    if (lengthInput && Number.isFinite(area.height_m)) lengthInput.value = area.height_m;
    if (swathInput && Number.isFinite(area.swath_m)) swathInput.value = area.swath_m;
    if (nameInput && area.name) nameInput.value = area.name;
  }
  openModal("workAreaModal");
  previewWorkArea();
}

function closeWorkArea() {
  if (areaPreviewLayer) areaPreviewLayer.setLatLngs([]);
  closeModal("workAreaModal");
}

function saveWorkArea() {
  const form = areaForm();
  showToast("Saving work area…");
  fetch(`${baseURL}/api/areas/rect`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      name: form.name,
      width_m: form.width,
      height_m: form.length,
      swath_m: form.swath,
    }),
  }).then((res) => res.json().then((data) => ({ ok: res.ok, data })))
    .then(({ ok, data }) => {
      if (!ok) throw new Error(data.error || "Could not set the work area");
      window.activeWorkArea = data.area;
      loadedAreaId = data.area.id;
      const areaPoints = data.area.map_polygon || [];
      workAreaLayer.setLatLngs(areaPoints);
      if (areaPoints.length >= 3 && leafletMap) {
        leafletMap.fitBounds(workAreaLayer.getBounds(), { padding: [24, 24], maxZoom: 18 });
      }
      closeWorkArea();
      updateAreaSummary(data.area);
      showToast(`Work area “${data.area.name}” set · ${form.width} × ${form.length} m, centred on the rover`);
    })
    .catch((error) => showToast(error.message));
}

function updateAutoProgressUI(prog) {
  if (autoProgressText) {
    let text = "Idle";
    if (prog.status === "fault") {
      text = `Fault · ${prog.fault || prog.message || "mission stopped"}`;
    } else if (prog.phase && prog.phase !== "idle") {
      const bits = [String(prog.phase).toUpperCase()];
      if (prog.total_strips) bits.push(`${prog.strip}/${prog.total_strips}`);
      bits.push(`${Math.round(prog.coverage_pct || 0)}%`);
      if (Array.isArray(prog.incomplete_strips) && prog.incomplete_strips.length) {
        bits.push(`skipped ${prog.incomplete_strips.length}`);
      }
      text = bits.join(" · ");
    }
    autoProgressText.innerText = text.toUpperCase();
  }
  if (autoProgressFill) {
    autoProgressFill.style.width = `${prog.coverage_pct || 0}%`;
  }
  if (missionPercent) missionPercent.innerText = `${Math.round(prog.coverage_pct || 0)}%`;
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

function updateAutoUI(isAutoActive) {
  roverAutoActive = Boolean(isAutoActive);
  const autoStatus = document.getElementById("autoStatus");
  const autoBtn = document.getElementById("autoModeBtn");
  if (autoStatus && autoBtn) {
    if (isAutoActive) {
      autoStatus.innerText = "STOP AUTONOMY";
      autoBtn.classList.add("active");
      if (modeTxt) modeTxt.innerText = "AUTONOMOUS 🤖";
    } else {
      autoStatus.innerText = "START AUTONOMY";
      autoBtn.classList.remove("active");
      if (modeTxt) modeTxt.innerText = "MANUAL 🎮";
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

function toggleAutoMode(action = "toggle") {
  const willStart = action === "start" || (action !== "stop" && !roverAutoActive);
  if (willStart && !window.activeWorkArea) {
    showToast("Set the work area breadth and length first");
    openWorkArea();
    return;
  }
  const sent = sendWSCommand({ type: "auto_mode", action: action });
  if (!sent) {
    roverFetch(`${baseURL}/auto_mode?action=${action}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success) {
          updateAutoUI(data.auto_active);
          showToast(data.auto_active ? "Autonomous mode started" : "Autonomous mode stopped");
        } else showToast(data.error || "Auto mode failed");
      })
      .catch(() => showToast("Could not reach rover (auto mode)"));
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

function updateAutoSpeed() {
  const speed = parseInt(autoSpeedSlider.value, 10);
  if (autoSpeedVal) autoSpeedVal.innerText = speed;
  const sent = sendWSCommand({ type: "auto_speed", speed_pct: speed });
  if (!sent) {
    roverFetch(`${baseURL}/auto_speed?speed_pct=${speed}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success) showToast(`Auto speed set to ${speed}%`);
        else showToast(data.error || "Could not set auto speed");
      })
      .catch(() => showToast("Could not reach rover (auto speed)"));
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
  } else if (key === "r" || key === "x") {
    toggleAutoMode();
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

