@echo off
cd /d C:\Users\niv\robot_project

echo Starting Robot System...

REM ── 1. Motor Daemon (C++ 200Hz) ──────────────────────────────────────────────
start "Motor Daemon" cmd /k "cd /d C:\Users\niv\robot_project && motor_daemon.exe --port COM4 --zmq-port 5555"

REM Wait for daemon to connect before starting app
timeout /t 2 /nobreak >nul

REM ── 2. Python App ────────────────────────────────────────────────────────────
start "Robot App" cmd /k "cd /d C:\Users\niv\robot_project\robot_sam2_app && E:\MiniForge\envs\lerobot\python.exe -m robot_sam2_app.main"

REM ── 3. Web Dashboard (optional — uncomment to enable) ────────────────────────
REM start "Dashboard" cmd /k "cd /d C:\Users\niv\robot_project && E:\MiniForge\envs\lerobot\python.exe -m uvicorn dashboard.backend.server:app --reload --port 8000"
REM start "" "http://localhost:8000"

echo All windows launched.
