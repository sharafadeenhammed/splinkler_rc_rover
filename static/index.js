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

// Use current page origin for dynamic IP support
const baseURL = window.location.origin;

let targetSpeed = 50;
let activeDirection = null; // Tracks active motor direction

if (slider) {
  slider.oninput = function () {
    targetSpeed = this.value;
    if (targetVal) targetVal.innerHTML = targetSpeed;
    if (activeDirection) {
      sendHardwareCommand(targetSpeed, activeDirection);
    }
  };
}

function sendHardwareCommand(speed, direction) {
  fetch(`${baseURL}/motor?speed=${speed}&dir=${direction}`)
    .then((res) => res.json())
    .then((data) => {
      if (data.success) {
        if (statusTxt) statusTxt.innerText = data.status;
      }
    })
    .catch((err) => console.error("Motor API error:", err));
}

function fetchSensorData() {
  fetch(`${baseURL}/sensor_readings`)
    .then((res) => res.json())
    .then((data) => {
      if (data.success) {
        if (valTemp) valTemp.innerText = data.temperature !== undefined ? `${data.temperature}°C` : '--';
        if (valHumidity) valHumidity.innerText = data.humidity !== undefined ? `${data.humidity}%` : '--';
        if (valSoil) valSoil.innerText = data.soil || 'Dry';
        if (valDist) valDist.innerText = data.distance_cm !== undefined ? `${data.distance_cm} cm` : '--';
        
        if (sensorReadings) {
          sensorReadings.innerHTML = `
            <p>Temp: ${data.temperature}°C</p>
            <p>Humidity: ${data.humidity}%</p>
            <p>Soil: ${data.soil}</p>
            <p>Dist: ${data.distance_cm}cm</p>
          `;
        }

        if (data.pump_active !== undefined) {
          updatePumpUI(data.pump_active);
        }
        if (data.sensor_deployed !== undefined) {
          updateSensorUI(data.sensor_deployed);
        }
        if (data.auto_active !== undefined) {
          updateAutoUI(data.auto_active);
        }
      }
    })
    .catch((err) => console.error("Sensor API error:", err));
}

function updateSensorUI(isDeployed) {
  const sensorStatus = document.getElementById("sensorStatus");
  const sensorBtn = document.getElementById("sensorDeployBtn");
  if (sensorStatus && sensorBtn) {
    if (isDeployed) {
      sensorStatus.innerText = "DEPLOYED (90°)";
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

async function toggleSensorDeploy() {
  try {
    const response = await fetch(`${baseURL}/deploy_sensor?action=toggle`);
    const data = await response.json();
    if (data.success) {
      updateSensorUI(data.is_deployed);
    }
  } catch (error) {
    console.error("Network error toggling sensor deploy:", error);
  }
}

async function toggleWaterPump() {
  try {
    const response = await fetch(`${baseURL}/water_pump?action=toggle`);
    const data = await response.json();
    if (data.success) {
      updatePumpUI(data.pump_active);
    }
  } catch (error) {
    console.error("Network error toggling water pump:", error);
  }
}

async function toggleAutoMode() {
  try {
    const response = await fetch(`${baseURL}/auto_mode?action=toggle`);
    const data = await response.json();
    if (data.success) {
      updateAutoUI(data.auto_active);
    }
  } catch (error) {
    console.error("Network error toggling auto mode:", error);
  }
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
      sendHardwareCommand(0, directionName);
    }
  };

  // Mouse events
  buttonElement.addEventListener("mousedown", startAction);
  buttonElement.addEventListener("mouseup", stopAction);
  buttonElement.addEventListener("mouseleave", stopAction);

  // Touch events
  buttonElement.addEventListener("touchstart", startAction);
  buttonElement.addEventListener("touchend", stopAction);
  buttonElement.addEventListener("touchcancel", stopAction);
}

// Bind logic rules to hardware buttons
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

// Camera pan tilt
async function tiltCamera(direction) {
  try {
    const response = await fetch(`${baseURL}/camera_tilt?direction=${direction}`);
    const data = await response.json();

    if (response.ok) {
      const angleElem = document.getElementById("current-angle");
      if (angleElem) angleElem.innerText = data.current_angle;
    }
  } catch (error) {
    console.error("Network error adjusting camera:", error);
  }
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

// Telemetry Polling Loop
setInterval(() => fetchSensorData(), 2000);
// Initial fetch
fetchSensorData();
