#include "DriftGameMode.h"

#include "DriftCarPawn.h"
#include "DriftHUD.h"
#include "Misc/ConfigCacheIni.h"

ADriftGameMode::ADriftGameMode()
{
	DefaultPawnClass = ADriftCarPawn::StaticClass();
	HUDClass = ADriftHUD::StaticClass();
}

APawn* ADriftGameMode::SpawnDefaultPawnAtTransform_Implementation(AController* NewPlayer, const FTransform& SpawnTransform)
{
	// Grid: alternating 3.5 m left/right, rows 8 m back.
	const int32 N = Spawned++;
	const FVector Right = SpawnTransform.GetRotation().GetRightVector();
	const FVector Back = -SpawnTransform.GetRotation().GetForwardVector();
	FTransform T = SpawnTransform;
	float Side = 350.f;
	GConfig->GetFloat(TEXT("DriftMultiplayer"), TEXT("GridSide"), Side, GGameIni);
	T.AddToTranslation(Right * ((N % 2) ? Side : 0.0) + Back * (N / 2) * 800.0);
	return Super::SpawnDefaultPawnAtTransform_Implementation(NewPlayer, T);
}
