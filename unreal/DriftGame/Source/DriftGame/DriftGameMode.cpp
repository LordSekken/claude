#include "DriftGameMode.h"

#include "DriftCarPawn.h"
#include "DriftHUD.h"

ADriftGameMode::ADriftGameMode()
{
	DefaultPawnClass = ADriftCarPawn::StaticClass();
	HUDClass = ADriftHUD::StaticClass();
}
