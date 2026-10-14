@echo off
rem Opens the DriftGame project in the Unreal Editor using DirectX 11.
rem Use this if the normal launch fails with a "GPU crash" (DXGI 887A002B).
start "" "C:\Program Files\Epic Games\UE_5.5\Engine\Binaries\Win64\UnrealEditor.exe" "%~dp0DriftGame.uproject" -dx11
