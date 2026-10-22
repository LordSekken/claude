// Steering wheel + force feedback through Drift Lab's helper (drift-sim/native/ffb):
// ffb_helper.exe (SDL3, DirectInput) reads the wheel and runs its force effects;
// this side starts it, receives its controls over UDP (localhost) and sends forces.
// Packets: "DLIN" controls in (port 47812), "DLST" status in, "DLFB" force out (47811).
#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"

class FSocket;

struct FDriftWheelMapping
{
	int32 SteerAxis = 0, ThrottleAxis = 1, BrakeAxis = 2, ClutchAxis = 3;
	float RotationDeg = 900.f;     // lock to lock
	bool bPedalsInverted = true;   // G920/G29: +1 released, -1 pressed
	int32 ShiftUp = 4, ShiftDown = 5, Handbrake = 0, Camera = 3, Reset = 2;
	int32 FirstGearButton = 12;    // H-shifter: gears 1..6 on consecutive buttons (-1 = none)
	int32 ReverseButton = 11;      // Driving Force Shifter on the Xbox G920: R comes first
	float FfbGain = 1.f;
	// Effects on top of the steering torque (like AC's road / kerb / slip settings, 0..1+).
	float FfbRoad = 0.5f, FfbKerb = 0.3f, FfbSlip = 0.15f;
	bool bFfbInvert = false;       // if the test pulls right first
	bool bTrace = false;           // helper logs forces + wheel angle to %TEMP%\DriftLab_ffb.log

	void Load();   // [DriftWheel] in Game.ini overrides the G920 defaults
};

class FDriftWheel
{
public:
	~FDriftWheel() { Stop(); }

	void Start();
	void Stop();
	void Poll();
	void SendForce(float Force, float Damper, float Spring, float VibAmp = 0.f, float VibHz = 0.f);
	void RunTest();

	bool HasControls() const { return FPlatformTime::Seconds() - LastInput < 0.5; }
	float Axis(int32 I) const { return I >= 0 && I < NumAxes ? Axes[I] : 0.f; }
	/** DirectInput reports exactly 0 until the first real poll - mid-travel on a pedal. */
	bool AxisLive(int32 I) const { return I >= 0 && I < 8 && ((LiveAxes >> I) & 1u); }
	bool Button(int32 I) const { return I >= 0 && I < 32 && (Buttons >> I) & 1u; }
	bool Pressed(int32 I) const { return Button(I) && !((PrevButtons >> I) & 1u); }
	uint32 ButtonBits() const { return Buttons; }
	const FString& Status() const { return StatusText; }

	FDriftWheelMapping Map;

private:
	FSocket* Rx = nullptr;
	FSocket* Tx = nullptr;
	FProcHandle Helper;
	float Axes[8] = {};
	int32 NumAxes = 0;
	uint32 Buttons = 0, PrevButtons = 0;
	uint32 LiveAxes = 0;
	double LastInput = -100.0;
	FString StatusText = TEXT("Wheel: starting helper...");
};
