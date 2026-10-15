@echo off
rem Starts DriftGame in VR (Quest 2 via Quest Link / Air Link: start Link first).
rem Cockpit view and the VR graphics preset are chosen automatically.
start "" "C:\Program Files\Epic Games\UE_5.5\Engine\Binaries\Win64\UnrealEditor.exe" "%~dp0DriftGame.uproject" -game -vr
