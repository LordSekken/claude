#include "DriftTelemetry.h"

#include <cmath>

#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "drift/Vehicle.h"

DEFINE_LOG_CATEGORY_STATIC(LogDriftTelemetry, Log, All);

namespace
{
	FString Slug(const FString& In)
	{
		FString Out;
		for (const TCHAR C : In)
		{
			Out.AppendChar(FChar::IsAlnum(C) || C == TEXT('-') ? FChar::ToLower(C) : TEXT('_'));
		}
		return Out.IsEmpty() ? TEXT("unknown") : Out.Left(60);
	}

	void Write(FArchive* F, const FString& S)
	{
		const FTCHARToUTF8 Utf8(*S);
		F->Serialize(const_cast<ANSICHAR*>(Utf8.Get()), Utf8.Length());
	}
}

bool FDriftTelemetry::Open(const FString& Car, const FString& Map, double InSteerScale)
{
	Close();
	SteerScale = InSteerScale;
	const FString Dir = FPaths::Combine(FPlatformProcess::UserDir(), TEXT("DriftLab/telemetry"));
	IFileManager::Get().MakeDirectory(*Dir, true);
	Path = FPaths::Combine(Dir, FString::Printf(TEXT("dg_%s_%s_%s.csv"), *Slug(Car), *Slug(Map),
		*FDateTime::Now().ToString(TEXT("%Y-%m-%d_%H-%M-%S"))));
	File = IFileManager::Get().CreateFileWriter(*Path);
	if (!File)
	{
		UE_LOG(LogDriftTelemetry, Warning, TEXT("Can't write %s"), *Path);
		return false;
	}
	Write(File, FString::Printf(TEXT("# DriftGame telemetry (AC layout and signs), car=%s map=%s\n"), *Car, *Map));
	FString Header = TEXT("t,speed_kmh,gear,rpm,gas,brake,clutch,steer,final_ff,heading,pitch,roll,"
		"vel_x,vel_y,vel_z,yaw_rate,pitch_rate,roll_rate,acc_x,acc_y,acc_z,pos_x,pos_y,pos_z");
	static const TCHAR* Wheels[4] = {TEXT("fl"), TEXT("fr"), TEXT("rl"), TEXT("rr")};
	for (const TCHAR* Per : {TEXT("slip"), TEXT("load"), TEXT("omega"), TEXT("susp"), TEXT("slip_angle"), TEXT("slip_ratio"), TEXT("fy"), TEXT("mz")})
	{
		for (const TCHAR* W : Wheels)
		{
			Header += FString::Printf(TEXT(",%s_%s"), Per, W);
		}
	}
	Write(File, Header + TEXT("\n"));
	Time = NextSample = 0.0;
	Rows = 0;
	UE_LOG(LogDriftTelemetry, Log, TEXT("Recording telemetry to %s"), *Path);
	return true;
}

void FDriftTelemetry::Sample(double DeltaSeconds, const drift::Vehicle& V, const FVector& WorldPos)
{
	if (!File)
	{
		return;
	}
	Time += DeltaSeconds;
	if (Time < NextSample)
	{
		return;
	}
	NextSample = FMath::Max(NextSample + 0.01, Time);
	// Body-local velocity / angular velocity. DriftCore: +x right, +y up, -z forward.
	// AC: +x left, +z forward, yaw rate + = turning left.
	const drift::Mat3& R = V.Rotation();
	const drift::Vec3 Vel = V.Velocity(), Ang = V.AngularVelocity();
	const drift::Vec3 Lv(R.c0.Dot(Vel), R.c1.Dot(Vel), R.c2.Dot(Vel));
	const drift::Vec3 La(R.c0.Dot(Ang), R.c1.Dot(Ang), R.c2.Dot(Ang));
	const drift::Vec3 Fwd = V.Forward();
	const double Heading = std::atan2(-Fwd.x, -Fwd.z);
	const drift::Inputs& In = V.in;
	FString Row = FString::Printf(TEXT("%.3f,%.2f,%d,%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,0,0,0,%.2f,%.2f,%.2f"),
		Rows * 0.01, V.Speed() * 3.6, V.Gear(), FMath::RoundToInt(V.Rpm()), In.throttle, In.brake, 1.0 - In.clutch,
		In.steer * SteerScale,
		-V.FfbForce(),   // AC: + pushes the rim left
		Heading, std::asin(FMath::Clamp(Fwd.y, -1.0, 1.0)), std::asin(FMath::Clamp(R.c0.y, -1.0, 1.0)),
		-Lv.x, Lv.y, -Lv.z, La.y, La.x, -La.z,
		WorldPos.X * 0.01, WorldPos.Y * 0.01, WorldPos.Z * 0.01);
	for (int32 K = 0; K < 8; ++K)
	{
		for (int32 I = 0; I < 4; ++I)
		{
			const drift::WheelState& W = V.Wheel(I);
			double Val = 0.0;
			switch (K)
			{
			case 0: Val = std::hypot(W.slip_ratio, std::tan(W.slip_angle)); break;
			case 1: Val = W.fz; break;
			case 2: Val = W.omega; break;
			case 3: Val = W.compression; break;
			case 4: Val = W.slip_angle; break;
			case 5: Val = W.slip_ratio; break;
			case 6: Val = W.fy; break;
			default: Val = 0.0; break;   // DriftCore has no per-tyre Mz
			}
			Row += FString::Printf(TEXT(",%.4f"), Val);
		}
	}
	Write(File, Row + TEXT("\n"));
	++Rows;
}

void FDriftTelemetry::Close()
{
	if (File)
	{
		File->Close();
		delete File;
		File = nullptr;
		UE_LOG(LogDriftTelemetry, Log, TEXT("Saved %d samples to %s"), Rows, *Path);
	}
}
