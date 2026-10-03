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

// New Auto Config Elements
const areaWidthInput = document.getElementById("areaWidthInput");
const areaHeightInput = document.getElementById("areaHeightInput");
const autoSpeedSlider = document.getElementById("autoSpeedSlider");
const autoSpeedVal = document.getElementById("autoSpeedVal");
const autoProgressText = document.getElementById("autoProgressText");
const autoProgressFill = document.getElementById("autoProgressFill");
const videoFeed = document.getElementById("videoFeed");
const valEncLeft = document.getElementById("valEncLeft");
const valEncRight = document.getElementById("valEncRight");

// Dynamic base URLs
const baseURL = window.location.origin;
const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
const wsURL = `${wsProtocol}//${window.location.host}/ws`;

let targetSpeed = 50;
let activeDirection = null; // Tracks active motor direction

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
  } else if (state === "connecting") {
    wsStatusTxt.innerText = text || "CONNECTING 🔄";
    wsStatusTxt.className = "connecting";
  } else {
    wsStatusTxt.innerText = text || "OFFLINE ⚠️";
    wsStatusTxt.className = "disconnected";
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
}

function updateAutoProgressUI(prog) {
  if (autoProgressText) {
    let text = "Idle";
    if (prog.phase !== "idle") {
      text = `${prog.phase.toUpperCase()} | Strip ${prog.strip}/${prog.total_strips} | ${prog.coverage_pct}%`;
    }
    autoProgressText.innerText = text;
  }
  if (autoProgressFill) {
    autoProgressFill.style.width = `${prog.coverage_pct || 0}%`;
  }
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
      autoStatus.innerText = "ON 🤖";
      autoBtn.classList.add("active");
      if (modeTxt) modeTxt.innerText = "AUTONOMOUS 🤖";
    } else {
      autoStatus.innerText = "OFF 🎮";
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

// ----------------- Autonomous Config Updaters -----------------

function updateAreaSettings() {
  const w = parseFloat(areaWidthInput.value);
  const h = parseFloat(areaHeightInput.value);
  const sent = sendWSCommand({ type: "set_area", width_m: w, height_m: h });
  if (!sent) {
    fetch(`${baseURL}/set_area?width_m=${w}&height_m=${h}`)
      .catch(err => console.error("Area HTTP error:", err));
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
    if (activeDirection) {
      sendHardwareCommand(targetSpeed, activeDirection);
    }
  };
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
