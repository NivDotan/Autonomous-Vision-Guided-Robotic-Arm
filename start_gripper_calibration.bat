@echo off
cd /d C:\Users\niv\robot_project

echo Starting Gripper Sensor Calibration...
E:\MiniForge\envs\lerobot\python.exe -u gripper_sensor_calibration.py %*

pause
