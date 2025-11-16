/**
 * IRIS Base Station Application
 * Main application logic for receiving and displaying lane detection data
 */

// Resolve ROSLIB in both browser (global) and Node environments
let ROSLIB;
if (typeof window !== "undefined" && window.ROSLIB) {
  ROSLIB = window.ROSLIB;
} else if (typeof require === "function") {
  ROSLIB = require("roslib");
} else {
  throw new Error("ROSLIB library not found. Make sure to load roslib.js before app.js");
}

class BaseStation {
  constructor() {
    // Connection state
    this.connected = false;
    this.connectionType = "websocket";
    this.ws = null;
    this.ros = null;

    // Canvas contexts
    this.rawCanvas = document.getElementById("rawCanvas");
    this.processedCanvas = document.getElementById("processedCanvas");
    this.rawCtx = this.rawCanvas.getContext("2d");
    this.processedCtx = this.processedCanvas.getContext("2d");

    // FPS tracking
    this.rawFpsCounter = new FPSCounter("rawFps");
    this.processedFpsCounter = new FPSCounter("processedFps");

    this.telemetryData = {
      steering_angle: 0,
      laneStatus: "Unknown",
      speed: 0,
      robotPosition: "Unknown",
      laneWidth: 0,
      deviation: 0,
      obstacleDetected: false,
      obstacleDistance: 0,
      obstaclePosition: "Unknown",
    };

    this.initializeEventListeners();
    this.log("System initialized. Ready to connect.", "info");
  }

  /**
   * Ngirim perintah kontrol (start/stop/reset)
   */
  sendCommand(command) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      const control_message = { type: "control", command };
      this.ws.send(JSON.stringify(control_message));
      this.log(`Sent command: ${command}`, "info");

      const statusEl = document.getElementById("statusMessage");
      if (statusEl) {
        if (command === "start") statusEl.textContent = "Sudah saatnya HIBECI jalan";
        else if (command === "stop") statusEl.textContent = "Stop sek";
        else if (command === "reset_distance") statusEl.textContent = "Reset dulu gak sih";
        statusEl.classList.remove("hidden");
      }
    } else {
      this.log(`Cannot send ${command}. WebSocket not connected.`, "error");
    }
  }

  /**
   * Event listeners setup
   */
  initializeEventListeners() {
    document.getElementById("connectionType").addEventListener("change", (e) => {
      this.connectionType = e.target.value;
      const rosSettings = document.getElementById("rosSettings");
      const serverUrlInput = document.getElementById("serverUrl");

      if (this.connectionType === "ros") {
        rosSettings.classList.remove("hidden");
        serverUrlInput.placeholder = "ws://localhost:9090";
        serverUrlInput.value = "ws://localhost:9090";
      } else {
        rosSettings.classList.add("hidden");
        serverUrlInput.placeholder = "ws://localhost:8080";
        serverUrlInput.value = "ws://localhost:8080";
      }
    });

    document.getElementById("connectBtn").addEventListener("click", () => this.connect());
    document.getElementById("disconnectBtn").addEventListener("click", () => this.disconnect());
    document.getElementById("clearLogsBtn").addEventListener("click", () => this.clearLogs());

    const startBtn = document.getElementById("startButton");
    const stopBtn = document.getElementById("stopButton");
    const resetBtn = document.getElementById("resetButton");

    if (startBtn) startBtn.addEventListener("click", () => this.sendCommand("start"));
    if (stopBtn) stopBtn.addEventListener("click", () => this.sendCommand("stop"));
    if (resetBtn) resetBtn.addEventListener("click", () => this.sendCommand("reset_distance"));
    
  }

  /**
   * Connect ke server (WebSocket atau ROS)
   */
  connect() {
    const serverUrl = document.getElementById("serverUrl").value;
    if (!serverUrl) return this.log("Please enter a server URL", "error");

    if (this.connectionType === "ros") this.connectROS(serverUrl);
    else this.connectWebSocket(serverUrl);
  }

  connectWebSocket(url) {
    this.log(`Connecting to WebSocket server: ${url}`, "info");
    try {
      this.ws = new WebSocket(url);
      this.ws.onopen = () => {
        this.connected = true;
        this.updateConnectionStatus(true);
        this.log("WebSocket connected successfully", "success");
      };

      this.ws.onmessage = (event) => this.handleWebSocketMessage(event.data);
      
      this.ws.onerror = (error) => this.log(`WebSocket error: ${error.message || "Connection failed"}`, "error");
      this.ws.onclose = () => {
        this.connected = false;
        this.updateConnectionStatus(false);
        this.log("WebSocket disconnected", "warning");
      };
    } catch (error) {
      this.log(`Failed to connect: ${error.message}`, "error");
    }
  }

  connectROS(url) {
    this.log(`Connecting to ROS bridge: ${url}`, "info");
    try {
      this.ros = new ROSLIB.Ros({ url });
      this.ros.on("connection", () => {
        this.connected = true;
        this.updateConnectionStatus(true);
        this.log("ROS bridge connected successfully", "success");
        this.subscribeToROSTopics();
      });
      this.ros.on("error", (error) => this.log(`ROS error: ${error}`, "error"));
      this.ros.on("close", () => {
        this.connected = false;
        this.updateConnectionStatus(false);
        this.log("ROS bridge disconnected", "warning");
      });
    } catch (error) {
      this.log(`Failed to connect to ROS: ${error.message}`, "error");
    }
  }

  /**
   * Handle WebSocket data
   */
  handleWebSocketMessage(data) {
    try {
      const message = JSON.parse(data);
      switch(message.type) {
      case "image_raw":
          this.displayImage(message.data, "raw");
          this.rawFpsCounter.tick();
          if (message.width && message.height) {
            document.getElementById(
              "rawResolution"
            ).textContent = `${message.width}x${message.height}`;
          }
          break;

        case "image_processed":
          this.displayImage(message.data, "processed");
          this.processedFpsCounter.tick();
          if (message.width && message.height) {
            document.getElementById(
              "processedResolution"
            ).textContent = `${message.width}x${message.height}`;
          }
          break;

        case "image_bev":
          this.displayImage(message.data, "bev");
          this.bevFpsCounter.tick();
          if (message.width && message.height)
            this.safeSetText("bevResolution", `${message.width}x${message.height}`);
          break;

        case "telemetry":
          this.updateTelemetry(message.data);
          break;

        case "steering_angle":
          this.updateTelemetry({ steering_angle: message.value });
          break;

        case "obstacle":
          this.updateTelemetry({
            obstacleDetected: message.detected,
            obstacleDistance: message.distance,
            obstaclePosition: message.position,
          });
          break;

        default:
          this.log(`Unknown message type: ${message.type}`, "warning");
      }
    } catch (error) {
      this.log(`Error parsing message: ${error.message}`, "error");
    }
  }

  displayImage(imageData, type) {
      const canvas = type === "raw" ? this.rawCanvas : this.processedCanvas;
      const ctx = type === "raw" ? this.rawCtx : this.processedCtx;
      const placeholder = document.getElementById(`${type}Placeholder`);

      if (placeholder) {
          placeholder.style.display = "none";
          canvas.style.display = "block";
      }

      const img = new Image();
      img.onload = () => {
          // Draw image scaled to existing canvas size
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
      };

      img.src = imageData.startsWith("data:image")
          ? imageData
          : `data:image/jpeg;base64,${imageData}`;
  }

  updateTelemetry(data) {
    Object.assign(this.telemetryData, data);

    if (data.steering_angle !== undefined) {
      this.safeSetText("steeringValue", `${data.steering_angle.toFixed(1)}°`);
      this.updateSteeringGauge(data.steering_angle);
    }

    if (data.laneStatus !== undefined) {
      const el = document.getElementById("laneStatus");
      if (el) {
        el.textContent = data.laneStatus;
        el.className =
          "badge " +
          (data.laneStatus === "Detected" ? "badge-success" : "badge-warning");
      }
    }

    if (data.robotPosition !== undefined) {
      const pos = document.getElementById("robotPosition");
      if (pos) {
        pos.textContent = data.robotPosition;
        pos.className =
          "badge " +
          (data.robotPosition === "center"
            ? "badge-success"
            : data.robotPosition === "left" || data.robotPosition === "right"
            ? "badge-warning"
            : "badge-secondary");
      }
    }

    if (data.speed !== undefined)
      this.safeSetText("speedValue", `${data.speed.toFixed(1)} cm/s`);

    if (data.obstacleDetected !== undefined) {
      const obs = document.getElementById("obstacleStatus");
      if (obs) {
        obs.textContent = data.obstacleDetected ? "Detected" : "None";
        obs.className =
          "badge " + (data.obstacleDetected ? "badge-error" : "badge-success");
      }
    }

    if (data.obstacleDistance !== undefined)
      this.safeSetText(
        "obstacleDistance",
        `${Number(data.obstacleDistance || 0).toFixed(0)} cm`
      );

    if (data.obstaclePosition !== undefined)
      this.safeSetText("obstaclePosition", data.obstaclePosition || "-");
  }

  updateSteeringGauge(angle) {
    const clamped = Math.max(-90, Math.min(90, angle));
    const offset = 125.6 - (clamped / 90) * 125.6;
    const valueEl = document.getElementById("gaugeValue");
    const needle = document.getElementById("gaugeNeedle");
    if (valueEl) valueEl.style.strokeDashoffset = offset;
    if (needle) needle.style.transform = `rotate(${clamped}deg)`;
  }

  disconnect() {
    if (this.ws) this.ws.close();
    if (this.ros) this.ros.close();
    this.connected = false;
    this.updateConnectionStatus(false);
    this.log("Disconnected from server", "info");
  }

  updateConnectionStatus(connected) {
    this.safeToggleClass("connectionStatus", connected ? "status-connected" : "status-disconnected");
    this.safeSetText("connectionText", connected ? "Connected" : "Disconnected");

    document.getElementById("connectBtn").disabled = connected;
    document.getElementById("disconnectBtn").disabled = !connected;
    ["startButton", "stopButton", "resetButton"].forEach((id) => {
      const btn = document.getElementById(id);
      if (btn) btn.disabled = !connected;
    });
  }

  log(message, type = "info") {
    const logContainer = document.getElementById("logContainer");
    if (!logContainer) return console.log(`[LOG] ${message}`);
    const timestamp = new Date().toLocaleTimeString();
    const entry = document.createElement("div");
    entry.className = "log-entry";
    entry.innerHTML = `<span class="log-timestamp">[${timestamp}]</span> <span class="log-${type}">${message}</span>`;
    logContainer.appendChild(entry);
    logContainer.scrollTop = logContainer.scrollHeight;
    console.log(`[${timestamp}] ${message}`);
  }

  clearLogs() {
    const logContainer = document.getElementById("logContainer");
    if (logContainer) logContainer.innerHTML = "";
    this.log("Logs cleared", "info");
  }

  // Safe DOM helpers
  safeSetText(id, text) {
    const el = document.getElementById(id);
    if (el) el.textContent = text;
  }

  safeToggleClass(id, className) {
    const el = document.getElementById(id);
    if (el) el.className = `status-indicator ${className}`;
  }
}

class FPSCounter {
  constructor(elementId) {
    this.elementId = elementId;
    this.frames = 0;
    this.lastTime = Date.now();
    setInterval(() => {
      const now = Date.now();
      const elapsed = (now - this.lastTime) / 1000;
      const fps = Math.round(this.frames / elapsed);
      this.frames = 0;
      this.lastTime = now;
      const el = document.getElementById(this.elementId);
      if (el) el.textContent = fps;
    }, 1000);
  }

  tick() {
    this.frames++;
  }
}

document.addEventListener("DOMContentLoaded", () => {
  window.baseStation = new BaseStation();
});
