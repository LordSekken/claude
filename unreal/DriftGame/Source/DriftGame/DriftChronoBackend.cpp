#include "DriftChronoBackend.h"

#include "DriftUnrealWorld.h"
#include "HAL/PlatformProcess.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"

#include "drift_chrono.h"

DEFINE_LOG_CATEGORY_STATIC(LogDriftChrono, Log, All);

namespace
{
	// Resolved once from DriftChrono.dll; the game never links against it.
	struct FApi
	{
		void* Dll = nullptr;
		bool bTried = false;
		decltype(&dc_version) Version = nullptr;
		decltype(&dc_create) Create = nullptr;
		decltype(&dc_destroy) Destroy = nullptr;
		decltype(&dc_set_ground) SetGround = nullptr;
		decltype(&dc_reset) Reset = nullptr;
		decltype(&dc_step) Step = nullptr;
		decltype(&dc_get_state) GetState = nullptr;
		decltype(&dc_last_step_us) LastStepUs = nullptr;
	};
	FApi GApi;

	bool LoadApi(FString& Err)
	{
		if (GApi.bTried)
		{
			if (!GApi.Dll)
			{
				Err = TEXT("DriftChrono.dll failed to load earlier");
			}
			return GApi.Dll != nullptr;
		}
		GApi.bTried = true;
		const FString Dir = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT("Binaries/Win64")));
		const FString Path = FPaths::Combine(Dir, TEXT("DriftChrono.dll"));
		if (!FPaths::FileExists(Path))
		{
			Err = FString::Printf(TEXT("%s not found"), *Path);
			return false;
		}
		// Chrono_core.dll / Chrono_vehicle.dll sit next to it.
		FPlatformProcess::PushDllDirectory(*Dir);
		GApi.Dll = FPlatformProcess::GetDllHandle(*Path);
		FPlatformProcess::PopDllDirectory(*Dir);
		if (!GApi.Dll)
		{
			Err = FString::Printf(TEXT("couldn't load %s (missing Chrono DLLs next to it?)"), *Path);
			return false;
		}
#define DC_GET(Field, Name) GApi.Field = reinterpret_cast<decltype(GApi.Field)>(FPlatformProcess::GetDllExport(GApi.Dll, TEXT(#Name)))
		DC_GET(Version, dc_version);
		DC_GET(Create, dc_create);
		DC_GET(Destroy, dc_destroy);
		DC_GET(SetGround, dc_set_ground);
		DC_GET(Reset, dc_reset);
		DC_GET(Step, dc_step);
		DC_GET(GetState, dc_get_state);
		DC_GET(LastStepUs, dc_last_step_us);
#undef DC_GET
		if (!GApi.Version || !GApi.Create || !GApi.Destroy || !GApi.SetGround || !GApi.Reset || !GApi.Step || !GApi.GetState
			|| !GApi.LastStepUs || GApi.Version() != DC_VERSION)
		{
			Err = TEXT("DriftChrono.dll is a different version than this game");
			FPlatformProcess::FreeDllHandle(GApi.Dll);
			GApi.Dll = nullptr;
			return false;
		}
		return true;
	}

	void ToArr(const drift::Vec3& V, double Out[3])
	{
		Out[0] = V.x;
		Out[1] = V.y;
		Out[2] = V.z;
	}

	drift::Vec3 FromArr(const double In[3]) { return drift::Vec3(In[0], In[1], In[2]); }
}

TUniquePtr<FDriftChrono> FDriftChrono::Create(const FString& CarId, FString& OutError)
{
	if (!LoadApi(OutError))
	{
		return nullptr;
	}
	const FString Root = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectContentDir(), TEXT("DriftData/ChronoCars")));
	char Err[512] = {};
	dc_car* Car = GApi.Create(TCHAR_TO_UTF8(*Root), TCHAR_TO_UTF8(*CarId), Err, sizeof(Err));
	if (!Car)
	{
		OutError = UTF8_TO_TCHAR(Err);
		return nullptr;
	}
	TUniquePtr<FDriftChrono> C(new FDriftChrono());
	C->Car = Car;
	GApi.SetGround(Car, &FDriftChrono::Raycast, C.Get());
	// [DriftChrono] FfbSource=AcCurve (default: AC's measured force-vs-front-slip curve on Chrono's
	// tyres) or Rack (Chrono's steering-rack load, FfbRackForce N = full force).
	FString Source = TEXT("Kingpin");
	GConfig->GetString(TEXT("DriftChrono"), TEXT("FfbSource"), Source, GGameIni);
	double Ref = 8000.0;
	GConfig->GetDouble(TEXT("DriftChrono"), TEXT("FfbRackForce"), Ref, GGameIni);
	C->RackForceRef = Source.Equals(TEXT("Rack"), ESearchCase::IgnoreCase) ? FMath::Max(Ref, 100.0) : 0.0;
	// Kingpin (default): the column torque from the front tyres' forces about each kingpin axis.
	double TorqueRef = 50.0;
	GConfig->GetDouble(TEXT("DriftChrono"), TEXT("FfbSteerTorque"), TorqueRef, GGameIni);
	C->SteerTorqueRef = Source.Equals(TEXT("Kingpin"), ESearchCase::IgnoreCase) ? FMath::Max(TorqueRef, 1.0) : 0.0;
	UE_LOG(LogDriftChrono, Log, TEXT("Chrono physics for %s"), *CarId);
	return C;
}

FDriftChrono::~FDriftChrono()
{
	if (Car && GApi.Destroy)
	{
		GApi.Destroy(Car);
	}
}

int FDriftChrono::Raycast(void* Ctx, const double From[3], const double To[3], double Point[3], double Normal[3], double* Grip)
{
	FDriftChrono* Self = static_cast<FDriftChrono*>(Ctx);
	if (!Self->World)
	{
		return 0;
	}
	const drift::GroundHit Hit = Self->World->Raycast(FromArr(From), FromArr(To));
	if (!Hit.hit)
	{
		return 0;
	}
	ToArr(Hit.point, Point);
	ToArr(Hit.normal, Normal);
	*Grip = drift::GetSurfaceInfo(Hit.surface).grip;
	return 1;
}

void FDriftChrono::Reset(const drift::Vec3& Origin, const drift::Mat3& Rot)
{
	double O[3], R[9];
	ToArr(Origin, O);
	ToArr(Rot.c0, R);
	ToArr(Rot.c1, R + 3);
	ToArr(Rot.c2, R + 6);
	GApi.Reset(Car, O, R);
}

void FDriftChrono::Step(double Dt, drift::Vehicle& Vehicle, FDriftUnrealWorld& InWorld)
{
	World = &InWorld;
	const drift::Vehicle::Controls Ctl = Vehicle.ControlStep(Dt);
	dc_controls C;
	C.steer_angle = Ctl.steer_angle;
	C.throttle = Ctl.throttle;
	C.brake = Ctl.brake;
	C.handbrake = Ctl.handbrake;
	C.clutch_engage = Ctl.engage;
	C.gear = Ctl.gear;

	// Walls and other cars touching the body box this frame.
	Contacts.clear();
	const drift::Mat3& Rot = Vehicle.Rotation();
	InWorld.BodyContacts(Vehicle.Origin() + Rot * Vehicle.CollisionCenterLocal(), Rot, Vehicle.CollisionHalfExtents(), Contacts);
	TArray<dc_contact, TInlineAllocator<16>> Out;
	const bool bUpsideDown = Rot.c1.y < 0.3;
	for (const drift::BodyContact& B : Contacts)
	{
		// Floors (kerbs, the road under a rolling body) are the tyres' job in Chrono; the body
		// only collides with walls and cars - and with the ground once it's on its side/roof.
		if (B.other_mass <= 0.0 && B.normal.y > 0.7 && !bUpsideDown)
		{
			continue;
		}
		dc_contact& D = Out.AddZeroed_GetRef();
		ToArr(B.point, D.point);
		ToArr(B.normal, D.normal);
		D.depth = B.depth;
		ToArr(B.other_vel, D.other_vel);
		D.other_mass = B.other_mass;
	}
	GApi.Step(Car, Dt, &C, Out.GetData(), Out.Num());
	LastMs = GApi.LastStepUs(Car) * 0.001;

	dc_state S;
	GApi.GetState(Car, &S);
	drift::Vehicle::ExternalState E;
	E.origin = FromArr(S.origin);
	E.rot.c0 = FromArr(S.rot);
	E.rot.c1 = FromArr(S.rot + 3);
	E.rot.c2 = FromArr(S.rot + 6);
	E.vel = FromArr(S.vel);
	E.ang_vel = FromArr(S.ang_vel);
	for (int I = 0; I < 4; ++I)
	{
		const dc_wheel& W = S.wheel[I];
		drift::Vehicle::ExternalWheel& X = E.wheel[I];
		X.center_local = FromArr(W.center_local);
		X.steer = W.steer;
		X.omega = W.omega;
		X.spin = W.spin;
		X.contact = W.contact != 0;
		X.point = FromArr(W.point);
		X.fz = W.fz;
		X.fx = W.fx;
		X.fy = W.fy;
		X.slip_ratio = W.slip_ratio;
		X.slip_angle = W.slip_angle;
	}
	E.rpm = S.rpm;
	E.rack_force = S.rack_force;
	E.rack_force_ref = RackForceRef;
	E.steer_torque = S.steer_torque;
	E.steer_torque_ref = SteerTorqueRef;
	Vehicle.ApplyExternal(E, Dt);
}
