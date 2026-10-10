@echo off
rem Starts Drift Lab in VR (OpenXR). Quest 2: connect with Quest Link / Air Link
rem or Virtual Desktop first, and make sure your OpenXR runtime is active.
cd /d "%~dp0"
start "" "DriftLab.exe" --xr-mode on
