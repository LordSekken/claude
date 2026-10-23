// Project Chrono car physics for the pawn, through DriftChrono.dll (unreal/DriftChrono).
// The DLL is loaded at runtime from Binaries/Win64 together with Chrono's own DLLs;
// if it or the car's Chrono files (Content/DriftData/ChronoCars/<car>/, written by
// drift-sim/tools/ac_to_chrono.py) are missing, Create() fails and the pawn keeps
// DriftCore. DriftCore still runs the driver aids and holds the state everything
// else reads (Vehicle::ControlStep / ApplyExternal).
#pragma once

#include "CoreMinimal.h"
#include "drift/Vehicle.h"

class FDriftUnrealWorld;
struct dc_car;

class FDriftChrono
{
public:
	static TUniquePtr<FDriftChrono> Create(const FString& CarId, FString& OutError);
	~FDriftChrono();

	void Reset(const drift::Vec3& Origin, const drift::Mat3& Rot);
	// One frame: driver aids from Vehicle, Chrono physics, state back into Vehicle.
	void Step(double Dt, drift::Vehicle& Vehicle, FDriftUnrealWorld& World);
	double LastStepMs() const { return LastMs; }
	// Rack force (N) for full force feedback ([DriftChrono] FfbSource=Rack); 0 = AC's FFB curve.
	double RackForceRef = 0.0;
	// Steering-column torque (N m) for full force feedback ([DriftChrono] FfbSource=Kingpin, FfbSteerTorque).
	double SteerTorqueRef = 0.0;
	double DefaultSteerTorqueRef = 0.0;   // Game.ini value at load: 100 % on the in-game strength readout
	double SteerTorqueExp = 1.4;          // [DriftChrono] FfbCurve

private:
	FDriftChrono() = default;
	dc_car* Car = nullptr;
	FDriftUnrealWorld* World = nullptr;
	double LastMs = 0.0;
	std::vector<drift::BodyContact> Contacts;

	static int Raycast(void* Ctx, const double From[3], const double To[3], double Point[3], double Normal[3], double* Grip);
};
