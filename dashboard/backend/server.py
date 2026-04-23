"""
FastAPI + WebSocket dashboard backend.

Broadcasts robot state at 10 Hz to all connected browser clients.
Also accepts click-to-grasp commands from the frontend.

Run:
    uvicorn dashboard.backend.server:app --reload --port 8000
"""
from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

app = FastAPI(title="Robot SAM2 Dashboard")

# Serve the frontend (static files).
_FRONTEND_DIR = Path(__file__).parent.parent / "frontend"
if _FRONTEND_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(_FRONTEND_DIR)), name="static")

# ── State broadcaster ─────────────────────────────────────────────────────────
_connections: list[WebSocket] = []
_latest_state: dict[str, Any] = {
    "joint_ticks": {},
    "tracking_active": False,
    "grasp_pose": None,
    "mode": "HAND",
    "motors_enabled": False,
    "timestamp": 0.0,
}


def update_state(state_dict: dict) -> None:
    """Called by the robot app to push new state (from a background thread)."""
    _latest_state.update(state_dict)
    _latest_state["timestamp"] = time.time()


async def _broadcast_loop() -> None:
    """Background task: push state to all WebSocket clients at 10 Hz."""
    while True:
        if _connections:
            msg = json.dumps(_latest_state)
            dead = []
            for ws in list(_connections):
                try:
                    await ws.send_text(msg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                _connections.remove(ws)
        await asyncio.sleep(0.1)  # 10 Hz


@app.on_event("startup")
async def startup() -> None:
    asyncio.create_task(_broadcast_loop())


# ── Routes ────────────────────────────────────────────────────────────────────

@app.get("/", response_class=HTMLResponse)
async def root() -> HTMLResponse:
    index = _FRONTEND_DIR / "index.html"
    if index.exists():
        return HTMLResponse(index.read_text())
    return HTMLResponse("<h1>Robot SAM2 Dashboard — frontend not found</h1>")


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    _connections.append(ws)
    try:
        while True:
            # Receive click-to-grasp commands from browser.
            data = await ws.receive_text()
            cmd = json.loads(data)
            if cmd.get("type") == "click_grasp":
                # Forward to robot app via a shared queue or callback.
                # (Integration point — wired in app.py via set_grasp_callback.)
                if _grasp_callback is not None:
                    _grasp_callback(cmd.get("x"), cmd.get("y"))
    except WebSocketDisconnect:
        _connections.remove(ws)


# ── Callback for browser-initiated grasps ────────────────────────────────────
_grasp_callback = None


def set_grasp_callback(cb) -> None:
    """Register a function the browser can call: cb(x_norm, y_norm)."""
    global _grasp_callback
    _grasp_callback = cb
