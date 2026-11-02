@echo off
cd /d "%~dp0"
title LoRa ROV Controller
echo Starting LoRa Controller...
python lora_controller.py
if errorlevel 1 (
    "C:\Users\chen\AppData\Local\Programs\Python\Python313\python.exe" lora_controller.py
)
if errorlevel 1 (
    echo.
    echo Process exited with error.
    pause
)
