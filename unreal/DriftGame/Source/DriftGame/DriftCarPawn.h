// The player's car. Project Chrono (DriftChronoBackend) or DriftCore simulates it;
// this actor just feeds it input and the level, then places the body,
// wheels and cameras where the simulation says.
#pragma once

#include <memory>

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "drift/Vehicle.h"
#include "DriftUnrealWorld.h"
#include "DriftTelemetry.h"
#include "DriftWheel.h"
#include "DriftChronoBackend.h"
#include "DriftCarPawn.generated.h"

/** What other players see of a car: sent ~30x/s by its owner, interpolated elsewhere. */
USTRUCT()
struct FDriftNetState
{
	GENERATED_BODY()

	UPROPERTY() FVector_NetQuantize10 Location;
	UPROPERTY() FRotator Rotation = FRotator::ZeroRotator;
	UPROPERTY() FVector_NetQuantize10 Velocity;
	UPROPERTY() float Steer = 0.f;                 // road-wheel angle, degrees
	UPROPERTY() float WheelOmega[4] = {0, 0, 0, 0}; // rad/s
	UPROPERTY() float Compression[4] = {0, 0, 0, 0};
	UPROPERTY() float Rpm = 0.f;
	UPROPERTY() int8 Gear = 0;
	UPROPERTY() float Time = 0.f;                 // owner's clock, s
};

class UCameraComponent;
class UInputAction;
class UInputMappingContext;
class USpringArmComponent;
class UStaticMeshComponent;
class UBoxComponent;

UCLASS()
class DRIFTGAME_API ADriftCarPawn : public APawn
{
	GENERATED_BODY()

public:
	ADriftCarPawn();

	virtual void Tick(float DeltaSeconds) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const override;
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

	/** Other cars' DriftCore bodies collide with this (query-only, ECC_Vehicle). */
	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<UBoxComponent> CarCollision;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<USpringArmComponent> ChaseArm;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<UCameraComponent> ChaseCamera;

	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<UCameraComponent> CockpitCamera;

	/** On the hood, just ahead of the windshield. */
	UPROPERTY(VisibleAnywhere, Category = "Drift")
	TObjectPtr<UCameraComponent> HoodCamera;

	// --- For the HUD -----------------------------------------------------------
	const drift::Vehicle* GetVehicle() const { return Vehicle.get(); }
	FString GetCarName() const { return CarName; }
	const FDriftWheel* GetWheel() const { return Wheel.get(); }
	/** Current velocity in cm/s - simulated (own car) or interpolated (others). */
	FVector GetCarVelocity() const;
	double GetCarMass() const;
	int32 GetGraphicsPreset() const { return GraphicsPreset; }
	/** "Chrono" or "DriftCore", and the physics time of the last frame. */
	FString GetPhysicsName() const { return Chrono ? TEXT("Chrono") : TEXT("DriftCore"); }
	double GetPhysicsMs() const { return Chrono ? Chrono->LastStepMs() : 0.0; }
	double GetPhysicsShownUntil() const { return PhysicsShownUntil; }
	double GetGraphicsShownUntil() const { return GraphicsShownUntil; }

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	void LoadSpec();
	void StartLocal();                  // first frame this machine drives the car
	void SendNetState();
	void ApplyRemote(float DeltaSeconds);
	UFUNCTION(Server, Unreliable) void ServerSetState(const FDriftNetState& State);
	UFUNCTION() void OnRep_NetState();
	void PushSnapshot(const FDriftNetState& State);
	bool LoadModel(const FString& ModelId);
	void ResetCar(const FVector& Location, float Yaw);
	void CreateInput();
	void ReadInput(float DeltaSeconds);
	bool ReadWheel();
	void SetCamera(int32 Mode);
	void SyncVisuals(float DeltaSeconds);

	void OnShiftUp();
	void OnShiftDown();
	void OnToggleCamera();
	void OnReset();
	void OnResetToSpawn();   // back to where the car started (wheel B, Backspace, gamepad Menu)
	void OnToggleAutoShift();
	void OnFfbTest();
	void OnCycleGraphics();
	void OnTogglePhysics();
	bool StartChrono();                 // false = stays on DriftCore (logged)

	std::unique_ptr<drift::Vehicle> Vehicle;
	std::unique_ptr<FDriftUnrealWorld> PhysWorld;
	std::unique_ptr<FDriftWheel> Wheel;
	std::unique_ptr<FDriftTelemetry> Telemetry;
	TUniquePtr<FDriftChrono> Chrono;    // Project Chrono physics; null = DriftCore
	double PhysicsShownUntil = 0.0;
	double NextPhysicsLog = 0.0;
	bool bAutoShotDone = false;
	bool bLocalStarted = false;
	double NextNetSend = 0.0;
	double NextNetLog = 0.0;
	UPROPERTY(ReplicatedUsing = OnRep_NetState) FDriftNetState NetState;
	struct FSnap { double Arrived; FDriftNetState S; };
	TArray<FSnap> Snaps;                // remote cars: recent states, oldest first
	double RemoteClockOffset = 0.0;     // their clock -> ours
	bool bHaveClock = false;
	float RemoteSpin[4] = {0, 0, 0, 0};
	FVector RemoteVelocity = FVector::ZeroVector;
	int32 CameraMode = 0;             // 0 chase, 1 cockpit, 2 hood
	float CockpitVFov = 54.f;         // vertical degrees, like AC (owner: 54)
	float HoodVFov = 54.f;
	float LastAspect = 0.f;
	int32 GraphicsPreset = 2;
	double GraphicsShownUntil = 0.0;
	bool bClutchUsed = false;         // clutch pedal touched: no auto clutch
	bool bShifterUsed = false;        // H-shifter touched: no gear = neutral
	FString CarName;
	bool bHasModel = false;           // real car meshes instead of the box
	FQuat WheelMeshBase = FQuat::Identity;
	FQuat ModelBase = FQuat::Identity;    // imported mesh frame -> DriftUE frame
	FVector SteerAxis = FVector::ForwardVector;   // Unreal frame, through the steering wheel hub
	double SteerRatio = 8.0;          // steering wheel turns per road-wheel turn
	FRotator ChaseRotation = FRotator::ZeroRotator;
	FVector LastOnRoad = FVector::ZeroVector;
	FVector SpawnLocation = FVector::ZeroVector;
	float SpawnYaw = 0.f;
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
	TObjectPtr<UInputAction> SpawnAction;
	UPROPERTY()
	TObjectPtr<UInputAction> AutoShiftAction;
	UPROPERTY()
	TObjectPtr<UInputAction> FfbTestAction;
	UPROPERTY()
	TObjectPtr<UInputAction> GraphicsAction;
	UPROPERTY()
	TObjectPtr<UInputAction> PhysicsAction;
};
