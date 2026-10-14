// The player's car. DriftCore simulates it (480 Hz, its own rigid body);
// this actor just feeds it input and the level, then places the body,
// wheels and cameras where the simulation says.
#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "drift/Vehicle.h"
#include "DriftUnrealWorld.h"
#include "DriftCarPawn.generated.h"

class UCameraComponent;
class UInputAction;
class UInputMappingContext;
class USpringArmComponent;
class UStaticMeshComponent;

UCLASS()
class DRIFTGAME_API ADriftCarPawn : public APawn
{
	GENERATED_BODY()

public:
	ADriftCarPawn();

	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	/** Car build to load from Content/DriftData/Cars/<CarSpec>.json (exported by Drift Lab). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift")
	FString CarSpec = TEXT("outback98_wrx_rwd_drift");

	/** Gamepad/keyboard: how strongly the front wheels self-align into a slide (0 = off). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift", meta = (ClampMin = "0", ClampMax = "1"))
	float SteeringAssist = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drift")
	bool bAutoShift = true;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<USceneComponent> Root;

	/** Placeholder body (scaled cube). Swap in your own car mesh in a Blueprint child. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drift")
	TObjectPtr<UStaticMeshComponent> Body;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TArray<TObjectPtr<USceneComponent>> WheelHubs;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TArray<TObjectPtr<UStaticMeshComponent>> WheelMeshes;

	/** Hub/brake parts of an imported car model: steer with the wheel, don't spin. */
	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TArray<TObjectPtr<UStaticMeshComponent>> SuspMeshes;

	/** Steering wheel of an imported car model (pivot at its hub). */
	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<UStaticMeshComponent> SteerMesh;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<USpringArmComponent> ChaseArm;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<UCameraComponent> ChaseCamera;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<UCameraComponent> CockpitCamera;

	// --- For the HUD -----------------------------------------------------------
	const drift::Vehicle* GetVehicle() const { return Vehicle.get(); }
	FString GetCarName() const { return CarName; }

protected:
	virtual void BeginPlay() override;

private:
	void LoadSpec();
	bool LoadModel(const FString& ModelId);
	void ResetCar(const FVector& Location, float Yaw);
	void CreateInput();
	void ReadInput(float DeltaSeconds);
	void SyncVisuals(float DeltaSeconds);

	void OnShiftUp();
	void OnShiftDown();
	void OnToggleCamera();
	void OnReset();
	void OnToggleAutoShift();

	std::unique_ptr<drift::Vehicle> Vehicle;
	std::unique_ptr<FDriftUnrealWorld> PhysWorld;
	FString CarName;
	bool bCockpit = false;
	bool bHasModel = false;           // real car meshes instead of the box
	FQuat WheelMeshBase = FQuat::Identity;
	FQuat ModelBase = FQuat::Identity;    // imported mesh frame -> DriftUE frame
	FVector SteerAxis = FVector::ForwardVector;   // Unreal frame, through the steering wheel hub
	double SteerRatio = 8.0;          // steering wheel turns per road-wheel turn
	FRotator ChaseRotation = FRotator::ZeroRotator;
	FVector LastOnRoad = FVector::ZeroVector;
	float LastOnRoadYaw = 0.f;
	double LowestY = -1000.0;   // metres, DriftCore frame

	UPROPERTY()
	TObjectPtr<UInputMappingContext> Mapping;
	UPROPERTY()
	TObjectPtr<UInputAction> SteerAction;
	UPROPERTY()
	TObjectPtr<UInputAction> ThrottleAction;
	UPROPERTY()
	TObjectPtr<UInputAction> BrakeAction;
	UPROPERTY()
	TObjectPtr<UInputAction> HandbrakeAction;
	UPROPERTY()
	TObjectPtr<UInputAction> ShiftUpAction;
	UPROPERTY()
	TObjectPtr<UInputAction> ShiftDownAction;
	UPROPERTY()
	TObjectPtr<UInputAction> CameraAction;
	UPROPERTY()
	TObjectPtr<UInputAction> ResetAction;
	UPROPERTY()
	TObjectPtr<UInputAction> AutoShiftAction;
};
