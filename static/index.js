const slider = document.getElementById("speedSlider");
const statusTxt = document.getElementById("statusTxt");
const targetVal = document.getElementById("targetVal");
const forwardBtn = document.getElementById("forwardBtn");
const reverseBtn = document.getElementById("reverseBtn");
const rightBtn = document.getElementById("rightBtn");
const leftBtn = document.getElementById("leftBtn");
const stopBtn = document.getElementById("stopBtn");
const sensorReadings = document.getElementById("sensorReadings");

const PI_IP = "192.168.161.181";

let targetSpeed = 50;
let activeDirection = null; // Tracks which direction is actively pressed

slider.oninput = function () {
  targetSpeed = this.value;
  targetVal.innerHTML = targetSpeed;
  if (activeDirection) {
    sendHardwareCommand(targetSpeed, activeDirection);
  }
};

const baseURL = `http://${PI_IP}:5000`;

function sendHardwareCommand(speed, direction) {
  fetch(`${baseURL}/motor?speed=${speed}&dir=${direction}`)
    .then((res) => res.json())
    .then((data) => {
      if (data.success) {
        statusTxt.innerText = data.status;
        if (speed > 0) {
          statusTxt.classList.add("active-status");
        } else {
          statusTxt.innerText = "STOPPED";
          statusTxt.classList.remove("active-status");
        }
      }
    })
    .catch((err) => console.error("API error:", err));
}

function fetchSensorData() {
  fetch(`http://${PI_IP}:5000/sensor_readings`)
    .then((res) => res.json())
    .then((data) => {
      /*
          <p style="text-align:left; font-size:12px">Updated at: ${date.getFullYear()}-${date.getMonth() + 1}-${date.getDate()} ${date.getHours()}:${date.getMinutes()}:${date.getSeconds()}
          */
      if (data.success) {
        // const date = new Date(Date(data.last_updated))
        sensorReadings.innerHTML = `
              <p style="text-align:left; font-size:12px">Temp: ${data.temperature} </p>
              <p style="text-align:left; font-size:12px">Humidity: ${data.humidity} </p>
              <p style="text-align:left; font-size:12px">Soil: ${data.soil} </p>
              <p style="text-align:left; font-size:12px">Updated at: ${data.last_updated}
           </p >
            `;
      }
    })
    .catch((err) => console.error("API error:", err));
}

function setupButton(buttonElement, directionName) {
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
setupButton(stopBtn, "stop");

// camera tilt
async function tiltCamera(direction) {
  try {
    const response = await fetch(
      `${baseURL}/camera_tilt?direction=${direction}`,
      {
        method: "get",
        headers: {
          "Content-Type": "application/json",
        },
        // body: JSON.stringify({ direction: direction }),
      },
    );

    const data = await response.json();

    if (response.ok) {
      // Update the text in the UI to match the new angle returned from Flask
      document.getElementById("current-angle").innerText = data.current_angle;
      console.log(`Successfully turned camera to: ${data.current_angle}°`);
    } else {
      console.error("Server Error:", data.message);
    }
  } catch (error) {
    console.error("Network error adjusting camera:", error);
  }
}

document.addEventListener("keydown", (event) => {
  // Only trigger if we aren't typing inside a text input field
  if (document.activeElement.tagName === "INPUT") return;

  if (event.key === "ArrowLeft" || event.key === "a") {
    tiltCamera("left");
  } else if (event.key === "ArrowRight" || event.key === "d") {
    tiltCamera("right");
  } else if (event.key === "c") {
    tiltCamera("center");
  }
});

setInterval(() => fetchSensorData(), 3000);
