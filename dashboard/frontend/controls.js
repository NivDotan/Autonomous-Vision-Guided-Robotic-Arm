/**
 * controls.js — WebSocket client and UI controls.
 *
 * Connects to ws://localhost:8000/ws, receives robot state JSON,
 * updates the Three.js arm visualisation and sidebar joint display.
 */

const WS_URL = `ws://${location.host}/ws`;
let ws = null;
let lastTs = 0, frameCount = 0;

const statusDot = document.getElementById("status-dot");
const hzLabel   = document.getElementById("hz-label");
const jointList = document.getElementById("joint-list");
const logDiv    = document.getElementById("log");
const ovMode    = document.getElementById("ov-mode");
const ovMotors  = document.getElementById("ov-motors");
const ovTracking= document.getElementById("ov-tracking");
const ovQuality = document.getElementById("ov-quality");

const MOTOR_NAMES = ["base","shoulder","elbow","palm","wrist","gripper"];
const TICK_MIN = 1000, TICK_MAX = 3000;

function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    statusDot.classList.add("connected");
    addLog("Connected to robot server.");
  };
  ws.onclose = () => {
    statusDot.classList.remove("connected");
    addLog("Disconnected — reconnecting in 2s...");
    setTimeout(connect, 2000);
  };
  ws.onerror = () => {};
  ws.onmessage = (evt) => {
    const state = JSON.parse(evt.data);
    onState(state);
  };
}

function onState(state) {
  // Update Hz counter.
  frameCount++;
  const now = performance.now();
  if (now - lastTs > 1000) {
    hzLabel.textContent = `${frameCount} Hz`;
    frameCount = 0;
    lastTs = now;
  }

  // Update 3D arm.
  if (window._updateArm && state.joint_ticks) {
    window._updateArm(state.joint_ticks);
  }

  // Update overlay.
  ovMode.textContent    = state.mode || "—";
  ovMotors.textContent  = state.motors_enabled ? "ON" : "OFF";
  ovTracking.textContent= state.tracking_active ? "YES" : "NO";
  const gp = state.grasp_pose;
  ovQuality.textContent = gp ? gp.quality?.toFixed(2) : "—";

  // Update joint bars.
  jointList.innerHTML = "";
  for (const name of MOTOR_NAMES) {
    const tick = (state.joint_ticks || {})[name] ?? 2048;
    const pct  = Math.round(((tick - TICK_MIN) / (TICK_MAX - TICK_MIN)) * 100);
    const row  = document.createElement("div");
    row.innerHTML = `
      <div class="joint-row">
        <span>${name}</span>
        <span style="color:#94a3b8">${tick}</span>
      </div>
      <div class="joint-bar">
        <div class="joint-fill" style="width:${Math.max(0,Math.min(100,pct))}%"></div>
      </div>`;
    jointList.appendChild(row);
  }
}

function addLog(msg) {
  const d = document.createElement("div");
  d.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
  logDiv.prepend(d);
  if (logDiv.children.length > 50) logDiv.lastChild.remove();
}

// ── Click-to-grasp from canvas ────────────────────────────────────────────────
document.getElementById("three-canvas").addEventListener("click", (e) => {
  const rect = e.target.getBoundingClientRect();
  const x = (e.clientX - rect.left) / rect.width;
  const y = (e.clientY - rect.top)  / rect.height;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "click_grasp", x, y }));
    addLog(`Grasp command sent: (${x.toFixed(3)}, ${y.toFixed(3)})`);
  }
});

connect();
