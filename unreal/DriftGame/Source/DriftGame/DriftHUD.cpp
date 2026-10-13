#include "DriftHUD.h"

#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "DriftCarPawn.h"

void ADriftHUD::DrawHUD()
{
	Super::DrawHUD();
	const ADriftCarPawn* Car = Cast<ADriftCarPawn>(GetOwningPawn());
	const drift::Vehicle* V = Car ? Car->GetVehicle() : nullptr;
	if (!V || !Canvas || !GEngine)
	{
		return;
	}
	UFont* Big = GEngine->GetLargeFont();
	UFont* Small = GEngine->GetSmallFont();
	const float X = 40.f;
	const float Y = Canvas->ClipY - 170.f;
	const FLinearColor White(0.95f, 0.95f, 0.95f);
	const FLinearColor Dim(0.6f, 0.6f, 0.6f);

	const int32 Gear = V->Gear();
	const FString GearText = Gear < 0 ? TEXT("R") : (Gear == 0 ? TEXT("N") : FString::FromInt(Gear));
	DrawText(FString::Printf(TEXT("%d km/h"), FMath::RoundToInt(V->Speed() * 3.6)), White, X, Y, Big, 2.0f);
	DrawText(GearText, White, X + 260.f, Y, Big, 2.0f);
	DrawText(Car->bAutoShift ? TEXT("AUTO") : TEXT("MANUAL"), Dim, X + 300.f, Y + 10.f, Small, 1.2f);

	// Rev bar, red past 90 % of the limiter.
	const double Limiter = V->Spec().limiter_rpm;
	const float Frac = FMath::Clamp(static_cast<float>(V->Rpm() / Limiter), 0.f, 1.f);
	const float BarW = 380.f;
	DrawRect(FLinearColor(0.1f, 0.1f, 0.1f, 0.7f), X, Y + 52.f, BarW, 12.f);
	DrawRect(Frac > 0.9f ? FLinearColor(0.95f, 0.15f, 0.1f) : FLinearColor(0.9f, 0.9f, 0.9f), X, Y + 52.f, BarW * Frac, 12.f);
	DrawText(FString::Printf(TEXT("%d rpm"), FMath::RoundToInt(V->Rpm())), Dim, X, Y + 70.f, Small, 1.2f);
	if (V->Spec().turbo)
	{
		DrawText(FString::Printf(TEXT("boost %.2f bar"), V->Boost()), Dim, X + 140.f, Y + 70.f, Small, 1.2f);
	}

	const double Angle = FMath::Abs(V->DriftAngleDeg());
	if (Angle > 5.0 && V->Speed() > 5.0)
	{
		DrawText(FString::Printf(TEXT("%d°"), FMath::RoundToInt(Angle)), FLinearColor(1.f, 0.75f, 0.2f), Canvas->ClipX * 0.5f - 40.f, 60.f, Big, 2.2f);
	}
	DrawText(Car->GetCarName(), Dim, X, Y + 100.f, Small, 1.2f);
	DrawText(TEXT("A/Space handbrake  B/E up  X/Q down  Y/C camera  View/R reset  RS/M auto-manual"), Dim, X, Y + 125.f, Small, 1.0f);
}
