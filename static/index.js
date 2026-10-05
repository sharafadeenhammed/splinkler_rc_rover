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
const missionAreaLabel = document.getElementById("missionAreaLabel");
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
const videoFeed = document.getElementById("videoFeed");
const valEncLeft = document.getElementById("valEncLeft");
const valEncRight = document.getElementById("valEncRight");

// Dynamic base URLs
const baseURL = window.location.origin;
const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
const wsURL = `${wsProtocol}//${window.location.host}/ws`;

let targetSpeed = 50;
let activeDirection = null; // Tracks active motor direction
const poseTrail = [];
let polygonDraft = [];
let mapDrawing = false;
let mapBounds = { min_x: -5, min_y: -5, max_x: 5, max_y: 5 };
let lastPose = null;

// WebSocket state
let ws = null;
let wsConnected = false;
let reconnectTimer = null;
let heartbeatInterval = null;
let httpPollInterval = null;
let videoRetryTimer = null;
let reconnectAttempts = 0;
const MAX_RECONNECT_DELAY_MS = 5000;

// Initialize video feed source
if (videoFeed) {
  videoFeed.src = `${baseURL}/stream`;
}

// ----------------- Video Feed Reconnection -----------------

function handleVideoError() {
  console.warn("📹 Video stream dropped. Retrying in 3 seconds...");
  clearTimeout(videoRetryTimer);
  videoRetryTimer = setTimeout(() => {
    if (videoFeed) {
      // Add cache buster to force reload
      videoFeed.src = `${baseURL}/stream?t=${new Date().getTime()}`;
    }
  }, 3000);
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

// ----------------- Telemetry Renderer -----------------

function renderTelemetry(data) {
  if (!data) return;

  if (valTemp) valTemp.innerText = data.temperature !== undefined ? `${data.temperature}°C` : '--';
  if (valHumidity) valHumidity.innerText = data.humidity !== undefined ? `${data.humidity}%` : '--';
  if (valSoil) valSoil.innerText = data.soil || 'Dry';
  if (valDist) valDist.innerText = data.distance_cm !== undefined ? `${data.distance_cm} cm` : '--';
  if (valEncLeft) valEncLeft.innerText = data.encoder_left !== undefined ? data.encoder_left : '0';
  if (valEncRight) valEncRight.innerText = data.encoder_right !== undefined ? data.encoder_right : '0';

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
  if (data.pose) renderRoverMap(data.pose, data.active_area);
  if (data.active_area && missionAreaLabel) {
    missionAreaLabel.innerText = `${data.active_area.name} · ${data.active_area.max_x - data.active_area.min_x} × ${data.active_area.max_y - data.active_area.min_y} m`;
  }
}

function renderRoverMap(pose, area) {
  if (!roverMap || !pose) return;
  const ctx = roverMap.getContext("2d");
  const w = roverMap.width;
  const h = roverMap.height;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#08111f";
  ctx.fillRect(0, 0, w, h);

  lastPose = pose;
  const bounds = area || { min_x: -5, min_y: -5, max_x: 5, max_y: 5 };
  mapBounds = bounds;
  if (area) window.activeWorkArea = area;
  const pad = 28;
  const spanX = Math.max(1, bounds.max_x - bounds.min_x);
  const spanY = Math.max(1, bounds.max_y - bounds.min_y);
  const scale = Math.min((w - pad * 2) / spanX, (h - pad * 2) / spanY);
  const sx = (x) => pad + (x - bounds.min_x) * scale;
  const sy = (y) => h - pad - (y - bounds.min_y) * scale;

  ctx.strokeStyle = "rgba(56,189,248,.8)";
  ctx.lineWidth = 2;
  if (area && Array.isArray(area.polygon) && area.polygon.length >= 3) {
    ctx.beginPath();
    area.polygon.forEach((point, i) => i ? ctx.lineTo(sx(point[0]), sy(point[1])) : ctx.moveTo(sx(point[0]), sy(point[1])));
    ctx.closePath();
    ctx.fillStyle = "rgba(57,217,208,.08)";
    ctx.fill();
    ctx.stroke();
  } else {
    ctx.strokeRect(sx(bounds.min_x), sy(bounds.max_y), spanX * scale, spanY * scale);
  }
  ctx.strokeStyle = "rgba(148,163,184,.18)";
  ctx.lineWidth = 1;
  for (let x = Math.ceil(bounds.min_x); x <= bounds.max_x; x++) {
    ctx.beginPath(); ctx.moveTo(sx(x), pad); ctx.lineTo(sx(x), h - pad); ctx.stroke();
  }
  for (let y = Math.ceil(bounds.min_y); y <= bounds.max_y; y++) {
    ctx.beginPath(); ctx.moveTo(pad, sy(y)); ctx.lineTo(w - pad, sy(y)); ctx.stroke();
  }

  poseTrail.push({ x: pose.x_m, y: pose.y_m });
  if (poseTrail.length > 300) poseTrail.shift();
  ctx.strokeStyle = "#a78bfa";
  ctx.lineWidth = 2;
  ctx.beginPath();
  poseTrail.forEach((p, i) => i ? ctx.lineTo(sx(p.x), sy(p.y)) : ctx.moveTo(sx(p.x), sy(p.y)));
  ctx.stroke();

  if (polygonDraft.length) {
    ctx.strokeStyle = "#f2ba66";
    ctx.fillStyle = "rgba(242,186,102,.12)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    polygonDraft.forEach((point, i) => i ? ctx.lineTo(sx(point[0]), sy(point[1])) : ctx.moveTo(sx(point[0]), sy(point[1])));
    if (polygonDraft.length >= 3) ctx.closePath();
    ctx.fill(); ctx.stroke();
    polygonDraft.forEach((point) => {
      ctx.fillStyle = "#f2ba66";
      ctx.beginPath(); ctx.arc(sx(point[0]), sy(point[1]), 4, 0, Math.PI * 2); ctx.fill();
    });
  }

  const rx = sx(pose.x_m), ry = sy(pose.y_m);
  const angle = -((pose.heading_deg || 0) * Math.PI / 180);
  ctx.save();
  ctx.translate(rx, ry); ctx.rotate(angle);
  ctx.fillStyle = "#22c55e";
  ctx.beginPath(); ctx.moveTo(12, 0); ctx.lineTo(-9, -7); ctx.lineTo(-6, 7); ctx.closePath(); ctx.fill();
  ctx.restore();
  ctx.fillStyle = "#e2e8f0";
  ctx.font = "12px sans-serif";
  ctx.fillText(`${pose.x_m.toFixed(2)} m, ${pose.y_m.toFixed(2)} m`, Math.min(w - 150, rx + 12), Math.max(14, ry - 12));
  if (poseText) poseText.innerText = `X ${pose.x_m.toFixed(2)} m · Y ${pose.y_m.toFixed(2)} m · ${pose.heading_deg.toFixed(1)}°`;
}

function mapToWorld(event) {
  const rect = roverMap.getBoundingClientRect();
  const px = (event.clientX - rect.left) * (roverMap.width / rect.width);
  const py = (event.clientY - rect.top) * (roverMap.height / rect.height);
  const pad = 28;
  const spanX = Math.max(1, mapBounds.max_x - mapBounds.min_x);
  const spanY = Math.max(1, mapBounds.max_y - mapBounds.min_y);
  const scale = Math.min((roverMap.width - pad * 2) / spanX, (roverMap.height - pad * 2) / spanY);
  return [
    Number((mapBounds.min_x + (px - pad) / scale).toFixed(2)),
    Number((mapBounds.min_y + (roverMap.height - pad - py) / scale).toFixed(2)),
  ];
}

function toggleMapDrawing() {
  mapDrawing = !mapDrawing;
  const button = document.getElementById("drawAreaBtn");
  const hint = document.getElementById("drawHint");
  if (button) button.classList.toggle("active", mapDrawing);
  if (hint) hint.innerText = mapDrawing ? "Click 3 or more points around the work area" : "Draw a boundary by clicking map points";
  if (roverMap) roverMap.classList.toggle("drawing", mapDrawing);
}

function undoPolygonPoint() {
  polygonDraft.pop();
  const hint = document.getElementById("drawHint");
  if (hint) hint.innerText = polygonDraft.length ? `${polygonDraft.length} point${polygonDraft.length === 1 ? "" : "s"} selected · add at least 3` : "Draw a boundary by clicking map points";
  if (lastPose) renderRoverMap(lastPose, window.activeWorkArea);
}

function clearPolygon() {
  polygonDraft = [];
  const hint = document.getElementById("drawHint");
  if (hint) hint.innerText = "Draw a boundary by clicking map points";
  if (lastPose) renderRoverMap(lastPose, window.activeWorkArea);
}

function savePolygonArea() {
  if (polygonDraft.length < 3) {
    showToast("Add at least three boundary points");
    return;
  }
  const name = (areaNameInput && areaNameInput.value) || "Map work area";
  fetch(`${baseURL}/api/areas/polygon`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name, points: polygonDraft }),
  }).then((res) => res.json().then((data) => ({ ok: res.ok, data })))
    .then(({ ok, data }) => {
      if (!ok) throw new Error(data.error || "Could not save boundary");
      polygonDraft = [];
      mapDrawing = false;
      const button = document.getElementById("drawAreaBtn");
      if (button) button.classList.remove("active");
      window.activeWorkArea = data.area;
      if (lastPose) renderRoverMap(lastPose, data.area);
      showToast("Multi-coordinate work area saved");
    })
    .catch((error) => showToast(error.message));
}

if (roverMap) {
  roverMap.addEventListener("click", (event) => {
    if (!mapDrawing) return;
    polygonDraft.push(mapToWorld(event));
    const hint = document.getElementById("drawHint");
    if (hint) hint.innerText = `${polygonDraft.length} point${polygonDraft.length === 1 ? "" : "s"} selected · add at least 3`;
    if (lastPose) renderRoverMap(lastPose, window.activeWorkArea);
  });
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
    fetch(`${baseURL}/motor?speed=${speed}&dir=${direction}`)
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
    fetch(`${baseURL}/deploy_sensor?action=${action}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success) updateSensorUI(data.is_deployed);
      })
      .catch((err) => console.error("Sensor deploy HTTP error:", err));
  }
}

function toggleWaterPump(action = "toggle") {
  const sent = sendWSCommand({ type: "pump", action: action });
  if (!sent) {
    fetch(`${baseURL}/water_pump?action=${action}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success) updatePumpUI(data.pump_active);
      })
      .catch((err) => console.error("Water pump HTTP error:", err));
  }
}

function toggleAutoMode(action = "toggle") {
  const sent = sendWSCommand({ type: "auto_mode", action: action });
  if (!sent) {
    fetch(`${baseURL}/auto_mode?action=${action}`)
      .then((res) => res.json())
      .then((data) => {
        if (data.success) updateAutoUI(data.auto_active);
      })
      .catch((err) => console.error("Auto mode HTTP error:", err));
  }
}

function tiltCamera(direction) {
  const sent = sendWSCommand({ type: "camera_tilt", direction: direction });
  if (!sent) {
    fetch(`${baseURL}/camera_tilt?direction=${direction}`)
      .then((res) => res.json())
      .then((data) => {
        const angleElem = document.getElementById("current-angle");
        if (angleElem && data.current_angle !== undefined) {
          angleElem.innerText = data.current_angle;
        }
      })
      .catch((err) => console.error("Camera tilt HTTP error:", err));
  }
}

function updateAutoSpeed() {
  const speed = parseInt(autoSpeedSlider.value);
  if (autoSpeedVal) autoSpeedVal.innerText = speed;
  const sent = sendWSCommand({ type: "auto_speed", speed_pct: speed });
  if (!sent) {
    fetch(`${baseURL}/auto_speed?speed_pct=${speed}`)
      .catch(err => console.error("Auto speed HTTP error:", err));
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
