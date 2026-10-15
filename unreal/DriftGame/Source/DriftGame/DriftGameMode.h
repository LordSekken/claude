#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "DriftGameMode.generated.h"

UCLASS()
class DRIFTGAME_API ADriftGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	ADriftGameMode();

	// Multiplayer: each new player starts beside/behind the last one, not on top.
	virtual APawn* SpawnDefaultPawnAtTransform_Implementation(AController* NewPlayer, const FTransform& SpawnTransform) override;

private:
	int32 Spawned = 0;
};
