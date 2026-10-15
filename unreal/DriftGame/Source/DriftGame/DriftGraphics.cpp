#include "DriftGraphics.h"

#include "GameFramework/GameUserSettings.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ConfigCacheIni.h"

namespace
{
	void SetCVar(const TCHAR* Name, float Value)
	{
		if (IConsoleVariable* V = IConsoleManager::Get().FindConsoleVariable(Name))
		{
			V->Set(Value, ECVF_SetByGameSetting);
		}
	}

	struct FPresetDef
	{
		const TCHAR* Name;
		int32 Scalability;        // 0 low .. 3 epic (sg.* groups)
		int32 GiMethod;           // 0 none, 1 Lumen
		int32 ReflectionMethod;   // 1 Lumen, 2 screen space
		int32 VirtualShadows;
		float ScreenPercentage;   // < 100: TSR upscales to the window
		int32 MotionBlur;
	};

	// VR on a Quest 2 + RTX 3070 Ti: two ~1832x1920 views at 72-90 fps, so no
	// Lumen or virtual shadows, and a lower internal resolution.
	const FPresetDef GPresets[DriftGraphics::Count] = {
		{TEXT("Low"), 0, 0, 2, 0, 67.f, 0},
		{TEXT("Medium"), 1, 1, 2, 0, 80.f, 1},
		{TEXT("High"), 2, 1, 1, 1, 100.f, 1},
		{TEXT("Epic"), 3, 1, 1, 1, 100.f, 1},
		{TEXT("VR"), 1, 0, 2, 0, 80.f, 0},
	};
}

const TCHAR* DriftGraphics::Name(int32 Preset)
{
	return GPresets[FMath::Clamp(Preset, 0, Count - 1)].Name;
}

int32 DriftGraphics::Saved()
{
	int32 Preset = High;
	GConfig->GetInt(TEXT("DriftGraphics"), TEXT("Preset"), Preset, GGameUserSettingsIni);
	return FMath::Clamp(Preset, 0, Count - 1);
}

void DriftGraphics::Apply(int32 Preset, bool bSave)
{
	const FPresetDef& P = GPresets[FMath::Clamp(Preset, 0, Count - 1)];
	if (UGameUserSettings* S = GEngine ? GEngine->GetGameUserSettings() : nullptr)
	{
		S->SetOverallScalabilityLevel(P.Scalability);
		S->ApplySettings(false);
	}
	SetCVar(TEXT("r.DynamicGlobalIlluminationMethod"), P.GiMethod);
	SetCVar(TEXT("r.ReflectionMethod"), P.ReflectionMethod);
	SetCVar(TEXT("r.Shadow.Virtual.Enable"), P.VirtualShadows);
	SetCVar(TEXT("r.ScreenPercentage"), P.ScreenPercentage);
	SetCVar(TEXT("r.MotionBlurQuality"), P.MotionBlur ? 4.f : 0.f);
	if (bSave)
	{
		GConfig->SetInt(TEXT("DriftGraphics"), TEXT("Preset"), Preset, GGameUserSettingsIni);
		GConfig->Flush(false, GGameUserSettingsIni);
	}
}

static FAutoConsoleCommand GDriftGraphicsCmd(
	TEXT("drift.Graphics"),
	TEXT("drift.Graphics <0-4>: Low, Medium, High, Epic, VR"),
	FConsoleCommandWithArgsDelegate::CreateLambda([](const TArray<FString>& Args) {
		if (Args.Num() > 0)
		{
			DriftGraphics::Apply(FCString::Atoi(*Args[0]));
		}
	}));
