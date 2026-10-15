@echo off
rem Hosts a multiplayer session on Shinamoyama. Friends join with "Join DriftGame.bat"
rem using your IP address (same network: your local IP; over the internet: forward UDP port 7777).
start "" "C:\Program Files\Epic Games\UE_5.5\Engine\Binaries\Win64\UnrealEditor.exe" "%~dp0DriftGame.uproject" "/Game/Maps/shinamoyama?listen" -game -windowed -ResX=1920 -ResY=1080
