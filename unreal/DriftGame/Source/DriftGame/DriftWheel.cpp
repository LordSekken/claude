#include "DriftWheel.h"

#include "Common/UdpSocketBuilder.h"
#include "Interfaces/IPv4/IPv4Address.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Paths.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

DEFINE_LOG_CATEGORY_STATIC(LogDriftWheel, Log, All);

namespace
{
	constexpr int32 HelperPort = 47811;
	constexpr int32 GamePort = 47812;

	TSharedRef<FInternetAddr> HelperAddr()
	{
		TSharedRef<FInternetAddr> Addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
		Addr->SetIp(FIPv4Address(127, 0, 0, 1).Value);
		Addr->SetPort(HelperPort);
		return Addr;
	}
}

void FDriftWheelMapping::Load()
{
	const TCHAR* S = TEXT("DriftWheel");
	GConfig->GetInt(S, TEXT("SteerAxis"), SteerAxis, GGameIni);
	GConfig->GetInt(S, TEXT("ThrottleAxis"), ThrottleAxis, GGameIni);
	GConfig->GetInt(S, TEXT("BrakeAxis"), BrakeAxis, GGameIni);
	GConfig->GetInt(S, TEXT("ClutchAxis"), ClutchAxis, GGameIni);
	GConfig->GetFloat(S, TEXT("RotationDeg"), RotationDeg, GGameIni);
	GConfig->GetBool(S, TEXT("PedalsInverted"), bPedalsInverted, GGameIni);
	GConfig->GetInt(S, TEXT("ShiftUpButton"), ShiftUp, GGameIni);
	GConfig->GetInt(S, TEXT("ShiftDownButton"), ShiftDown, GGameIni);
	GConfig->GetInt(S, TEXT("HandbrakeButton"), Handbrake, GGameIni);
	GConfig->GetInt(S, TEXT("CameraButton"), Camera, GGameIni);
	GConfig->GetInt(S, TEXT("ResetButton"), Reset, GGameIni);
	GConfig->GetInt(S, TEXT("FirstGearButton"), FirstGearButton, GGameIni);
	GConfig->GetFloat(S, TEXT("FfbGain"), FfbGain, GGameIni);
	GConfig->GetBool(S, TEXT("FfbInvert"), bFfbInvert, GGameIni);
}

void FDriftWheel::Start()
{
	Map.Load();
	Rx = FUdpSocketBuilder(TEXT("DriftWheelRx")).AsNonBlocking().AsReusable()
		.BoundToAddress(FIPv4Address(127, 0, 0, 1)).BoundToPort(GamePort).Build();
	Tx = FUdpSocketBuilder(TEXT("DriftWheelTx")).AsNonBlocking().Build();
	if (!Rx)
	{
		StatusText = FString::Printf(TEXT("Wheel: port %d busy (is Drift Lab running?)"), GamePort);
	}
	// Next to the game's binaries with SDL3.dll: the project's Binaries/Win64 in
	// the editor, the exe's folder in a packaged game.
	FString Exe = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectDir(), TEXT("Binaries/Win64/ffb_helper.exe")));
	if (!FPaths::FileExists(Exe))
	{
		Exe = FPaths::ConvertRelativePathToFull(FPaths::Combine(FPlatformProcess::BaseDir(), TEXT("ffb_helper.exe")));
	}
	if (!FPaths::FileExists(Exe))
	{
		StatusText = TEXT("Wheel: ffb_helper.exe missing from Binaries/Win64 (build it from drift-sim/native/ffb)");
		return;
	}
	const FString Args = FString::Printf(TEXT("--parent %u"), FPlatformProcess::GetCurrentProcessId());
	Helper = FPlatformProcess::CreateProc(*Exe, *Args, true, true, true, nullptr, 0, nullptr, nullptr);
	if (!Helper.IsValid())
	{
		StatusText = TEXT("Wheel: couldn't start ffb_helper.exe (blocked by antivirus?)");
	}
}

void FDriftWheel::Stop()
{
	if (Tx)
	{
		int32 Sent = 0;
		Tx->SendTo(reinterpret_cast<const uint8*>("DLQT"), 4, Sent, *HelperAddr());
	}
	for (FSocket** S : {&Rx, &Tx})
	{
		if (*S)
		{
			(*S)->Close();
			ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(*S);
			*S = nullptr;
		}
	}
	if (Helper.IsValid())
	{
		FPlatformProcess::CloseProc(Helper);   // it quits on DLQT, or when we exit
	}
}

void FDriftWheel::Poll()
{
	PrevButtons = Buttons;
	if (!Rx)
	{
		return;
	}
	uint8 Buf[512];
	int32 Read = 0;
	uint32 Pending = 0;
	while (Rx->HasPendingData(Pending) && Rx->Recv(Buf, sizeof(Buf), Read) && Read >= 4)
	{
		if (FMemory::Memcmp(Buf, "DLIN", 4) == 0 && Read >= 8)
		{
			const int32 Na = FMath::Min<int32>(Buf[4], 8);
			if (Read < 8 + Na * 4 + 4)
			{
				continue;
			}
			NumAxes = Na;
			FMemory::Memcpy(Axes, Buf + 8, Na * 4);
			for (int32 A = 0; A < Na; ++A)
			{
				LiveAxes |= Axes[A] != 0.f ? (1u << A) : 0u;
			}
			FMemory::Memcpy(&Buttons, Buf + 8 + Na * 4, 4);
			LastInput = FPlatformTime::Seconds();
		}
		else if (FMemory::Memcmp(Buf, "DLST", 4) == 0)
		{
			const FString NewStatus(Read - 4, UTF8_TO_TCHAR(reinterpret_cast<const char*>(Buf + 4)));
			if (NewStatus != StatusText)
			{
				UE_LOG(LogDriftWheel, Log, TEXT("%s"), *NewStatus);
			}
			StatusText = NewStatus;
		}
	}
}

void FDriftWheel::SendForce(float Force, float Damper)
{
	if (!Tx)
	{
		return;
	}
	// "DLFB" force(-1..1, +right) damper spring gain
	uint8 Buf[20];
	FMemory::Memcpy(Buf, "DLFB", 4);
	const float Vals[4] = {FMath::Clamp(Map.bFfbInvert ? -Force : Force, -1.f, 1.f), FMath::Clamp(Damper, 0.f, 1.f), 0.f, Map.FfbGain};
	FMemory::Memcpy(Buf + 4, Vals, sizeof(Vals));
	int32 Sent = 0;
	Tx->SendTo(Buf, sizeof(Buf), Sent, *HelperAddr());
}

void FDriftWheel::RunTest()
{
	if (Tx)
	{
		int32 Sent = 0;
		Tx->SendTo(reinterpret_cast<const uint8*>("DLTS"), 4, Sent, *HelperAddr());
	}
}
