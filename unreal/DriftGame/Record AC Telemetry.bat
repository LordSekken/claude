@echo off
rem Records Assetto Corsa telemetry (speed, steering, force feedback, slip...) while you drive.
rem Files go to Documents\DriftLab\telemetry. Close the window to stop.
start "AC telemetry recorder" "%~dp0Binaries\Win64\ac_telemetry.exe"
