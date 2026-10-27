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
const sidebar = document.getElementById("sidebar");
const mobileBackdrop = document.getElementById("mobileBackdrop");
const sidebarClock = document.getElementById("sidebarClock");
const missionPercent = document.getElementById("missionPercent");
const missionActionLabel = document.getElementById("missionActionLabel");
const throttleOutput = document.getElementById("throttleOutput");

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
const videoFeed = document.getElementById("videoFeed");
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

// Command PIN: the field appears in the sidebar only when the server
// enforces one (telemetry pin_required), and the value persists locally.
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
        json: () => Promise.resolve({ success: false, error: "PIN required — set it in the sidebar" }),
      };
    }
    return res;
  });
}

let targetSpeed = 50;
let activeDirection = null; // Tracks active motor direction
let polygonDraft = [];
let mapDrawing = false;
const mapOriginDefault = [6.5244, 3.3792];
let mapOrigin = mapOriginDefault.slice();
let lastPose = null;
let leafletMap = null;
let roverMarker = null;
let workAreaLayer = null;
let draftLayer = null;
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
let videoRetryTimer = null;
let reconnectAttempts = 0;
let videoRetryAttempts = 0;
const MAX_RECONNECT_DELAY_MS = 5000;

// Initialize video feed source
if (videoFeed) videoFeed.src = `${baseURL}/stream`;

if (roverMap && window.L) {
  leafletMap = L.map(roverMap, { zoomControl: true, scrollWheelZoom: true, wheelPxPerZoomLevel: 100 }).setView(mapOrigin, 18);
  L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  }).addTo(leafletMap);
  workAreaLayer = L.polygon([], { color: "#38d9cf", weight: 2, fillColor: "#38d9cf", fillOpacity: 0.13 }).addTo(leafletMap);
  draftLayer = L.polygon([], { color: "#f2ba66", weight: 2, dashArray: "5 5", fillColor: "#f2ba66", fillOpacity: 0.12 }).addTo(leafletMap);
  trailLayer = L.polyline([], { color: "#a78bfa", weight: 3, opacity: 0.8 }).addTo(leafletMap);
  roverMarker = L.marker(mapOrigin, { icon: L.divIcon({ className: "rover-map-marker", html: "<span>▲</span>", iconSize: [28, 28], iconAnchor: [14, 14] }) }).addTo(leafletMap);
  leafletMap.on("click", (event) => {
    if (!mapDrawing || polygonDraft.length >= 128) return;
    polygonDraft.push([event.latlng.lat, event.latlng.lng]);
    draftLayer.setLatLngs(polygonDraft);
    if (mapCoordinateText) mapCoordinateText.innerText = `${event.latlng.lat.toFixed(5)}, ${event.latlng.lng.toFixed(5)}`;
    updateDrawHint();
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

// ----------------- Video Feed Reconnection -----------------

function handleVideoError() {
  console.warn("📹 Video stream dropped. Retrying with backoff...");
  clearTimeout(videoRetryTimer);
  const delay = Math.min(3000 * (2 ** videoRetryAttempts), 30000);
  videoRetryAttempts = Math.min(videoRetryAttempts + 1, 4);
  videoRetryTimer = setTimeout(() => {
    if (videoFeed) {
      videoFeed.src = `${baseURL}/stream`;
    }
  }, delay);
}

if (videoFeed) {
  videoFeed.addEventListener("load", () => { videoRetryAttempts = 0; });
}

// ----------------- Connection Badge & State -----------------

function updateConnectionBadge(state, text) {
  if (!wsStatusTxt) return;
  if (state === "connected") {
    wsStatusTxt.innerText = text || "ONLINE ⚡";
    wsStatusTxt.className = "connected";
    if (connectionDot) connectionDot.className = "status-dot online";
  } else if (state === "connecting") {
    wsStatusTxt.innerText = text || "CONNECTING 🔄";
    wsStatusTxt.className = "connecting";
    if (connectionDot) connectionDot.className = "status-dot";
  } else {
    wsStatusTxt.innerText = text || "OFFLINE ⚠️";
    wsStatusTxt.className = "disconnected";
    if (connectionDot) connectionDot.className = "status-dot";
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
    console.warn(`🔌 [WS] Disconnected (code: ${event.code}). Server stopped rover actions.`);
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
// Aviation pattern: the vehicle crate stays fixed in the middle, the card
// rotates behind it under a fixed lubber line.  Pose/IMU heading is math
// style (0° = east, CCW); the compass shows 0° = north, clockwise.

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
    const pitch = data.imu.pitch_deg || 0;
    const roll = data.imu.roll_deg || 0;
    const model = document.getElementById("rover3dModel");
    if (model) {
      // Attitude only — heading lives on the compass card now.
      model.style.transform = `rotateX(${-pitch}deg) rotateY(${roll}deg)`;
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

function toggleMapDrawing() {
  mapDrawing = !mapDrawing;
  const button = document.getElementById("drawAreaBtn");
  const hint = document.getElementById("drawHint");
  if (mapDrawing) {
    centerMapOnRover();
    if (!originSetForWork && !loadedOrigin) {
      showToast("Set origin at rover first, then draw the boundary");
    } else {
      showToast("Tap map corners — drag to pan when not placing points");
    }
  }
  if (button) button.classList.toggle("active", mapDrawing);
  if (hint) {
    hint.innerText = mapDrawing
      ? "Tap boundary points · pan with drag"
      : "Origin at rover centres the map · then draw boundary";
  }
  if (leafletMap) leafletMap.getContainer().classList.toggle("drawing", mapDrawing);
}

function updateDrawHint() {
  const hint = document.getElementById("drawHint");
  if (hint) hint.innerText = `${polygonDraft.length} points · need 3+ · drag map to pan`;
  draftLayer.setLatLngs(polygonDraft);
}

function undoPolygonPoint() {
  if (!polygonDraft.length) {
    showToast("Nothing to undo");
    return;
  }
  polygonDraft.pop();
  updateDrawHint();
  showToast("Removed last boundary point");
}

function clearPolygon() {
  polygonDraft = [];
  updateDrawHint();
  showToast("Boundary draft cleared");
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

function savePolygonArea() {
  if (polygonDraft.length < 3) {
    showToast("Add at least three boundary points");
    return;
  }
  if (!originSetForWork) {
    showToast("Tap “Origin here” at the rover before saving");
    return;
  }
  showToast("Saving work area…");
  const name = (areaNameInput && areaNameInput.value) || "Map work area";
  const localPoints = polygonDraft.map((point) => geoToLocal(point));
  fetch(`${baseURL}/api/areas/polygon`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, points: localPoints, map_points: polygonDraft, origin: mapOrigin }),
  }).then((res) => res.json().then((data) => ({ ok: res.ok, data })))
    .then(({ ok, data }) => {
      if (!ok) throw new Error(data.error || "Could not save boundary");
      polygonDraft = [];
      mapDrawing = false;
      const button = document.getElementById("drawAreaBtn");
      if (button) button.classList.remove("active");
      window.activeWorkArea = data.area;
      loadedAreaId = data.area.id;
      workAreaLayer.setLatLngs(data.area.map_polygon || []);
      draftLayer.setLatLngs([]);
      if (lastPose) renderRoverMap(lastPose, data.area);
      showToast(`Work area “${data.area.name || name}” saved`);
    })
    .catch((error) => showToast(error.message));
}

function updateAutoProgressUI(prog) {
  if (autoProgressText) {
    let text = "Idle";
    if (prog.phase !== "idle") {
      text = `${prog.phase.toUpperCase()} | Strip ${prog.strip}/${prog.total_strips} | ${prog.coverage_pct}%`;
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
  if (missionActionLabel) missionActionLabel.innerText = isAutoActive ? "Stop autonomous mission" : "Start autonomous mission";
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

function toggleSidebar(open) {
  if (sidebar) sidebar.classList.toggle("open", open);
  if (mobileBackdrop) mobileBackdrop.classList.toggle("open", open);
  if (open && leafletMap) setTimeout(() => leafletMap.invalidateSize(), 280);
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

setInterval(() => {
  if (sidebarClock) sidebarClock.innerText = new Date().toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" });
}, 1000);

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
  clearTimeout(videoRetryTimer);
  if (activeDirection) {
    sendHardwareCommand(0, "stop");
  }
  if (ws) {
    try {
      ws.close();
    } catch (_) {}
  }
});

// Initialize WebSocket on page load
initWebSocket();

