@echo off
rem Plays DriftGame without the editor around it (full speed, fullscreen window).
start "" "C:\Program Files\Epic Games\UE_5.5\Engine\Binaries\Win64\UnrealEditor.exe" "%~dp0DriftGame.uproject" -game -windowed -ResX=2560 -ResY=1440
