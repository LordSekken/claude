#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "DriftHUD.generated.h"

UCLASS()
class DRIFTGAME_API ADriftHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;
};
