// Records DriftGame telemetry in the same CSV layout as drift-sim/native/actelemetry
// (Assetto Corsa's shared memory), with AC's sign conventions, so a DriftGame run
// and an AC run of the same car/corner can be compared directly.
// Files: Documents\DriftLab\telemetry\dg_<car>_<map>_<date>.csv, 100 Hz.
// On when [DriftTelemetry] Record=True in Game.ini.
#pragma once

#include "CoreMinimal.h"

namespace drift { class Vehicle; }

class FDriftTelemetry
{
public:
	~FDriftTelemetry() { Close(); }

	bool Open(const FString& Car, const FString& Map, double SteerScale);
	void Sample(double DeltaSeconds, const drift::Vehicle& V, const FVector& WorldPos);
	void Close();
	bool IsOpen() const { return File != nullptr; }

private:
	FArchive* File = nullptr;
	double Time = 0.0, NextSample = 0.0;
	double SteerScale = 1.0;   // DriftCore steer (fraction of lock) -> AC steer (fraction of 450 deg)
	int32 Rows = 0;
	FString Path;
};
