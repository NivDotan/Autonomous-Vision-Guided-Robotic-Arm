@echo off
cd /d C:\Users\niv\robot_project

echo Starting Robot System V2 (bottom-center aim, current-motor grasp)...

REM -- 1. Motor Daemon (C++ 200Hz) ----------------------------------------------
start "Motor Daemon V2" cmd /k "cd /d C:\Users\niv\robot_project && motor_daemon.exe --port COM4 --zmq-port 5555"

REM Wait for daemon to connect before starting app
timeout /t 2 /nobreak >nul

REM -- 2. Python App V2 ---------------------------------------------------------
start "Robot App V2" cmd /k "cd /d C:\Users\niv\robot_project\robot_sam2_app_v2 && E:\MiniForge\envs\lerobot\python.exe -m robot_sam2_app.main"

echo All windows launched.
