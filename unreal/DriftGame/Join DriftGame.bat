@echo off
rem Joins a friend's DriftGame session.
set /p HOSTIP=Host IP address (e.g. 192.168.1.20):
start "" "C:\Program Files\Epic Games\UE_5.5\Engine\Binaries\Win64\UnrealEditor.exe" "%~dp0DriftGame.uproject" %HOSTIP% -game -windowed -ResX=1920 -ResY=1080
