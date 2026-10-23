#include "DriftCarPawn.h"

#include <string>

#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "HeadMountedDisplayFunctionLibrary.h"
#include "Components/StaticMeshComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "EnhancedPlayerInput.h"
#include "Engine/GameViewportClient.h"
#include "Engine/Engine.h"
#include "UnrealClient.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "UObject/ConstructorHelpers.h"
#include "drift/CarSpec.h"
#include "drift/Json.h"
#include "DriftGraphics.h"
#include "DriftChronoBackend.h"
#include "DriftUE.h"
#include "HAL/IConsoleManager.h"
#include "Net/UnrealNetwork.h"

// Test autopilot for headless runs: >= 0 overrides the pedals / steering of the local car.
static TAutoConsoleVariable<float> CVarAutoThrottle(TEXT("drift.AutoThrottle"), -1.f, TEXT("Test: throttle 0..1 (-1 = off)"));
static TAutoConsoleVariable<float> CVarAutoSteer(TEXT("drift.AutoSteer"), -2.f, TEXT("Test: steering -1..1 (-2 = off)"));
static TAutoConsoleVariable<float> CVarAutoShot(TEXT("drift.AutoShot"), 0.f, TEXT("Test: take a HighResShot this many seconds into play (0 = off)"));
static TAutoConsoleVariable<float> CVarAutoQuit(TEXT("drift.AutoQuit"), 0.f, TEXT("Test: quit this many seconds into play (0 = off)"));
static TAutoConsoleVariable<int32> CVarPhysicsLog(TEXT("drift.PhysicsLog"), 0, TEXT("Log car state once a second"));
static TAutoConsoleVariable<int32> CVarNoWheel(TEXT("drift.NoWheel"), 0, TEXT("1 = don't start the wheel/FFB helper (unattended tests)"));
static TAutoConsoleVariable<int32> CVarUseDriftCore(TEXT("drift.UseDriftCore"), 0, TEXT("1 = start on DriftCore physics instead of Chrono (set before Play)"));

DEFINE_LOG_CATEGORY_STATIC(LogDrift, Log, All);
DEFINE_LOG_CATEGORY_STATIC(LogDriftNet, Log, All);

ADriftCarPawn::ADriftCarPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	// Multiplayer: each player's car is simulated on their own machine and its
	// state replicated (FDriftNetState); Unreal's movement replication is off.
	bReplicates = true;
	SetReplicatingMovement(false);
	bAlwaysRelevant = true;
	SetNetUpdateFrequency(30.f);
	// No AutoPossessPlayer: the game mode gives each player their car (with it
	// set, every car spawned on a host grabbed the host's own controller).

	Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	RootComponent = Root;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));

	// The simulation does all collision itself; the visuals never collide.
	Body = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Body"));
	Body->SetupAttachment(Root);
	Body->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	if (CubeMesh.Succeeded())
	{
		Body->SetStaticMesh(CubeMesh.Object);
	}

	static const TCHAR* WheelNames[4] = {TEXT("WheelFL"), TEXT("WheelFR"), TEXT("WheelRL"), TEXT("WheelRR")};
	for (int32 I = 0; I < 4; ++I)
	{
		USceneComponent* Hub = CreateDefaultSubobject<USceneComponent>(*FString::Printf(TEXT("%sHub"), WheelNames[I]));
		Hub->SetupAttachment(Root);
		UStaticMeshComponent* WheelMesh = CreateDefaultSubobject<UStaticMeshComponent>(WheelNames[I]);
		WheelMesh->SetupAttachment(Hub);
		WheelMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (CylinderMesh.Succeeded())
		{
			WheelMesh->SetStaticMesh(CylinderMesh.Object);
		}
		// Engine cylinder: 1 m tall along Z, 1 m across. Roll it onto its side.
		WheelMesh->SetRelativeRotation(FRotator(0.f, 0.f, 90.f));
		UStaticMeshComponent* Susp = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("%sSusp"), WheelNames[I]));
		Susp->SetupAttachment(Hub);
		Susp->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		WheelHubs.Add(Hub);
		WheelMeshes.Add(WheelMesh);
		SuspMeshes.Add(Susp);
	}
	WheelMeshBase = FRotator(0.f, 0.f, 90.f).Quaternion();

	CarCollision = CreateDefaultSubobject<UBoxComponent>(TEXT("CarCollision"));
	CarCollision->SetupAttachment(Root);
	CarCollision->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	CarCollision->SetCollisionObjectType(ECC_Vehicle);
	CarCollision->SetCollisionResponseToAllChannels(ECR_Ignore);
	CarCollision->SetGenerateOverlapEvents(false);

	SteerMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("SteeringWheel"));
	SteerMesh->SetupAttachment(Root);
	SteerMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	ChaseArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("ChaseArm"));
	ChaseArm->SetupAttachment(Root);
	ChaseArm->SetUsingAbsoluteRotation(true);   // we steer it toward the direction of travel
	ChaseArm->TargetArmLength = 620.f;
	ChaseArm->SocketOffset = FVector(0.f, 0.f, 120.f);
	ChaseArm->bDoCollisionTest = true;
	ChaseArm->bEnableCameraLag = false;

	ChaseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
	ChaseCamera->SetupAttachment(ChaseArm, USpringArmComponent::SocketName);
	ChaseCamera->SetFieldOfView(80.f);

	CockpitCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("CockpitCamera"));
	CockpitCamera->SetupAttachment(Root);
	CockpitCamera->SetFieldOfView(75.f);
	CockpitCamera->SetAutoActivate(false);

	HoodCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("HoodCamera"));
	HoodCamera->SetupAttachment(Root);
	HoodCamera->SetFieldOfView(80.f);
	HoodCamera->SetAutoActivate(false);
}

void ADriftCarPawn::BeginPlay()
{
	Super::BeginPlay();
	LoadSpec();
	PhysWorld = std::make_unique<FDriftUnrealWorld>(GetWorld(), this);
	// The rest needs to know this machine drives the car, which a joining
	// client learns a moment after BeginPlay - see StartLocal.
}

void ADriftCarPawn::StartLocal()
{
	bLocalStarted = true;
	// [DriftPhysics] Backend=Chrono (default) or DriftCore; F6 switches while driving.
	FString Backend = TEXT("Chrono");
	GConfig->GetString(TEXT("DriftPhysics"), TEXT("Backend"), Backend, GGameIni);
	if (!Backend.Equals(TEXT("DriftCore"), ESearchCase::IgnoreCase) && CVarUseDriftCore.GetValueOnGameThread() == 0)
	{
		StartChrono();
	}
	ResetCar(GetActorLocation(), GetActorRotation().Yaw);
	LastOnRoad = SpawnLocation = GetActorLocation();
	LastOnRoadYaw = SpawnYaw = GetActorRotation().Yaw;
	LowestY = Vehicle->Origin().y - 300.0;
	ChaseRotation = FRotator(-9.f, GetActorRotation().Yaw, 0.f);
	if (CVarNoWheel.GetValueOnGameThread() == 0 && !FParse::Param(FCommandLine::Get(), TEXT("NoWheel")))
	{
		Wheel = std::make_unique<FDriftWheel>();
		Wheel->Start();
	}
	GraphicsPreset = DriftGraphics::Saved();
	if (UHeadMountedDisplayFunctionLibrary::IsHeadMountedDisplayEnabled())
	{
		// VR: sit in the cockpit (the headset moves the camera) and use the VR
		// preset - two eyes at Quest 2 resolution need the lighter settings.
		GraphicsPreset = DriftGraphics::VR;
		SetCamera(1);
		UHeadMountedDisplayFunctionLibrary::ResetOrientationAndPosition();
	}
	DriftGraphics::Apply(GraphicsPreset, false);
	GConfig->GetFloat(TEXT("DriftCamera"), TEXT("CockpitVFov"), CockpitVFov, GGameIni);
	GConfig->GetFloat(TEXT("DriftCamera"), TEXT("HoodVFov"), HoodVFov, GGameIni);
	bool bRecord = false;
	GConfig->GetBool(TEXT("DriftTelemetry"), TEXT("Record"), bRecord, GGameIni);
	if (bRecord)
	{
		// Steering in AC's units: fraction of a 450 deg (lock-to-centre) wheel.
		const double LockDeg = FMath::RadiansToDegrees(Vehicle->Spec().steer_lock);
		Telemetry = std::make_unique<FDriftTelemetry>();
		Telemetry->Open(CarName, UWorld::RemovePIEPrefix(GetWorld()->GetMapName()), LockDeg * SteerRatio / 450.0);
	}
}

void ADriftCarPawn::EndPlay(const EEndPlayReason::Type Reason)
{
	Wheel.reset();   // stops the helper: the wheel goes limp
	Telemetry.reset();
	Super::EndPlay(Reason);
}

void ADriftCarPawn::LoadSpec()
{
	drift::CarSpec Spec;
	FLinearColor Paint(0.55f, 0.55f, 0.58f);
	FString ModelId;
	bool bHoodSet = false;
	SteerRatio = 0.0;
	// Scripts/build_car.py picks the car to drive (an imported AC car).
	FString Selected;
	if (FFileHelper::LoadFileToString(Selected, *FPaths::Combine(FPaths::ProjectContentDir(), TEXT("DriftData/SelectedCar.txt"))))
	{
		Selected.TrimStartAndEndInline();
		if (!Selected.IsEmpty())
		{
			CarSpec = Selected;
		}
	}
	const FString Path = FPaths::Combine(FPaths::ProjectContentDir(), TEXT("DriftData/Cars"), CarSpec + TEXT(".json"));
	FString Text;
	if (FFileHelper::LoadFileToString(Text, *Path))
	{
		const std::string Utf8(TCHAR_TO_UTF8(*Text));
		std::string Err;
		if (drift::CarSpec::FromJson(Utf8, Spec, &Err))
		{
			drift::JsonValue Json;
			drift::JsonValue::Parse(Utf8, Json, nullptr);
			const drift::JsonValue& S = Json.Has("spec") ? Json["spec"] : Json;
			const FString Hex = UTF8_TO_TCHAR(S.Str("color", "#8c8c94").c_str());
			Paint = FLinearColor(FColor::FromHex(Hex));
			if (S.Has("model"))
			{
				const drift::JsonValue& M = S["model"];
				ModelId = UTF8_TO_TCHAR(M.Str("id", "").c_str());
				SteerRatio = M.Num("steer_ratio", 0.0);
				const drift::JsonValue& P = M["steer_pivot"];
				const drift::JsonValue& A = M["steer_axis"];
				if (M.Bool("has_steer", false) && P.IsArray() && P.arr.size() == 3 && A.IsArray() && A.arr.size() == 3)
				{
					SteerMesh->SetRelativeLocation(DriftUE::Pos(drift::Vec3(P.arr[0].num, P.arr[1].num, P.arr[2].num)));
					SteerAxis = DriftUE::Dir(drift::Vec3(A.arr[0].num, A.arr[1].num, A.arr[2].num)).GetSafeNormal();
				}
				const drift::JsonValue& H = M["hood_cam"];
				if (H.IsArray() && H.arr.size() == 3)
				{
					HoodCamera->SetRelativeLocation(DriftUE::Pos(drift::Vec3(H.arr[0].num, H.arr[1].num, H.arr[2].num)));
					bHoodSet = true;
				}
			}
		}
		else
		{
			UE_LOG(LogDrift, Error, TEXT("Bad car spec %s: %s"), *Path, UTF8_TO_TCHAR(Err.c_str()));
		}
	}
	else
	{
		UE_LOG(LogDrift, Warning, TEXT("Car spec %s not found - using a generic car."), *Path);
	}
	CarName = UTF8_TO_TCHAR(Spec.name.c_str());
	Vehicle = std::make_unique<drift::Vehicle>(Spec);
	CockpitCamera->SetRelativeLocation(DriftUE::Pos(Vehicle->SeatLocal()));
	{
		const drift::Vec3 Half = Vehicle->CollisionHalfExtents();
		CarCollision->SetRelativeLocation(DriftUE::Pos(Vehicle->CollisionCenterLocal()));
		CarCollision->SetBoxExtent(FVector(Half.z, Half.x, Half.y) * 100.0);
	}
	if (!bHoodSet)
	{
		// No model: centreline, a bit below eye height, ~1.1 m ahead of the seat.
		const drift::Vec3 Seat = Vehicle->SeatLocal();
		HoodCamera->SetRelativeLocation(DriftUE::Pos(drift::Vec3(0.0, Seat.y - 0.08, Seat.z - 1.1)));
	}
	if (SteerRatio <= 1.0)
	{
		// Without the car's own ratio: full wheel rotation = full lock.
		SteerRatio = 450.0 / FMath::Max(10.0, FMath::RadiansToDegrees(Spec.steer_lock));
	}
	bHasModel = !ModelId.IsEmpty() && LoadModel(ModelId);
	if (bHasModel)
	{
		return;
	}

	// Size the placeholder body and wheels to the spec.
	const drift::Vec3 Half = Vehicle->CollisionHalfExtents();
	Body->SetRelativeLocation(DriftUE::Pos(Vehicle->CollisionCenterLocal()));
	Body->SetRelativeScale3D(FVector(Half.z, Half.x, Half.y) * 2.0);
	if (UMaterialInstanceDynamic* Mid = Body->CreateDynamicMaterialInstance(0))
	{
		Mid->SetVectorParameterValue(TEXT("Color"), Paint);
	}
	for (int32 I = 0; I < 4; ++I)
	{
		const double D = Vehicle->Wheel(I).radius * 2.0;
		WheelMeshes[I]->SetRelativeScale3D(FVector(D, D, 0.22));
		if (UMaterialInstanceDynamic* Mid = WheelMeshes[I]->CreateDynamicMaterialInstance(0))
		{
			Mid->SetVectorParameterValue(TEXT("Color"), FLinearColor(0.03f, 0.03f, 0.03f));
		}
	}
}

bool ADriftCarPawn::LoadModel(const FString& ModelId)
{
	// Meshes from Scripts/build_car.py, already in DriftCore's body frame
	// (wheel parts centred on the wheel, the steering wheel on its hub).
	auto Part = [&ModelId](const TCHAR* Name) {
		return LoadObject<UStaticMesh>(nullptr, *FString::Printf(TEXT("/Game/Cars/%s/%s.%s"), *ModelId, Name, Name));
	};
	UStaticMesh* BodyMesh = Part(TEXT("BODY"));
	if (!BodyMesh)
	{
		UE_LOG(LogDrift, Warning, TEXT("Car model /Game/Cars/%s not imported - run Scripts/build_car.py. Using the box."), *ModelId);
		return false;
	}
	// Unreal's glTF import maps (x, y, z) to (x, z, y); DriftUE expects
	// (-z, x, y). The two differ by a +90 deg yaw (no mirroring).
	const FQuat ImportFix = FRotator(0.f, 90.f, 0.f).Quaternion();
	Body->SetStaticMesh(BodyMesh);
	Body->SetRelativeLocation(FVector::ZeroVector);
	Body->SetRelativeRotation(ImportFix);
	Body->SetRelativeScale3D(FVector::OneVector);
	ModelBase = ImportFix;
	static const TCHAR* Wheels[4] = {TEXT("LF"), TEXT("RF"), TEXT("LR"), TEXT("RR")};
	for (int32 I = 0; I < 4; ++I)
	{
		WheelMeshes[I]->SetStaticMesh(Part(*FString::Printf(TEXT("WHEEL_%s"), Wheels[I])));
		WheelMeshes[I]->SetRelativeScale3D(FVector::OneVector);
		SuspMeshes[I]->SetStaticMesh(Part(*FString::Printf(TEXT("SUSP_%s"), Wheels[I])));
		SuspMeshes[I]->SetRelativeRotation(ImportFix);
	}
	WheelMeshBase = ImportFix;
	SteerMesh->SetStaticMesh(Part(TEXT("STEER")));
	UE_LOG(LogDrift, Log, TEXT("Using car model /Game/Cars/%s"), *ModelId);
	return true;
}

void ADriftCarPawn::ResetCar(const FVector& Location, float Yaw)
{
	if (!Vehicle || !PhysWorld)
	{
		return;
	}
	// Drop onto whatever is below: body origin sits at axle height.
	drift::Vec3 Origin = DriftUE::CorePos(Location);
	const drift::GroundHit Ground = PhysWorld->Raycast(Origin + drift::Vec3(0, 3, 0), Origin - drift::Vec3(0, 50, 0));
	if (Ground.hit)
	{
		const drift::WheelState& W = Vehicle->Wheel(0);
		Origin = Ground.point + drift::Vec3(0, W.radius + W.travel * 0.6, 0);
	}
	Vehicle->Reset(Origin, DriftUE::CoreRot(FRotator(0.f, Yaw, 0.f).Quaternion()));
	if (Chrono)
	{
		Chrono->Reset(Vehicle->Origin(), Vehicle->Rotation());
	}
	SetActorLocationAndRotation(DriftUE::Pos(Vehicle->Origin()), DriftUE::Rot(Vehicle->Rotation()));
}

bool ADriftCarPawn::StartChrono()
{
	FString Err;
	Chrono = FDriftChrono::Create(CarSpec, Err);
	if (!Chrono)
	{
		UE_LOG(LogDrift, Warning, TEXT("Chrono physics unavailable (%s) - using DriftCore."), *Err);
		return false;
	}
	return true;
}

void ADriftCarPawn::OnTogglePhysics()
{
	if (!Vehicle)
	{
		return;
	}
	const drift::Vec3 Origin = Vehicle->Origin();
	const drift::Mat3 Rot = Vehicle->Rotation();
	const drift::Vec3 Vel = Vehicle->Velocity(), AngVel = Vehicle->AngularVelocity();
	if (Chrono)
	{
		// Back to DriftCore, keeping the car's motion.
		Chrono.Reset();
		Vehicle->Reset(Origin, Rot);
		Vehicle->SetMotion(Vel, AngVel);
	}
	else if (StartChrono())
	{
		Chrono->Reset(Origin + Rot.c1 * 0.02, Rot);   // starts at rest
	}
	PhysicsShownUntil = FPlatformTime::Seconds() + 3.0;
}

static void MapAxis(UInputMappingContext* Ctx, UInputAction* Action, const FKey& Key, bool bNegate = false, float DeadZone = 0.f)
{
	FEnhancedActionKeyMapping& M = Ctx->MapKey(Action, Key);
	if (DeadZone > 0.f)
	{
		UInputModifierDeadZone* Dz = NewObject<UInputModifierDeadZone>(Ctx);
		Dz->LowerThreshold = DeadZone;
		Dz->Type = EDeadZoneType::Axial;
		M.Modifiers.Add(Dz);
	}
	if (bNegate)
	{
		M.Modifiers.Add(NewObject<UInputModifierNegate>(Ctx));
	}
}

void ADriftCarPawn::CreateInput()
{
	if (Mapping)
	{
		return;
	}
	auto MakeAction = [this](const TCHAR* Name, EInputActionValueType Type) {
		UInputAction* A = NewObject<UInputAction>(this, FName(Name));
		A->ValueType = Type;
		return A;
	};
	SteerAction = MakeAction(TEXT("IA_Steer"), EInputActionValueType::Axis1D);
	ThrottleAction = MakeAction(TEXT("IA_Throttle"), EInputActionValueType::Axis1D);
	BrakeAction = MakeAction(TEXT("IA_Brake"), EInputActionValueType::Axis1D);
	HandbrakeAction = MakeAction(TEXT("IA_Handbrake"), EInputActionValueType::Axis1D);
	ShiftUpAction = MakeAction(TEXT("IA_ShiftUp"), EInputActionValueType::Boolean);
	ShiftDownAction = MakeAction(TEXT("IA_ShiftDown"), EInputActionValueType::Boolean);
	CameraAction = MakeAction(TEXT("IA_Camera"), EInputActionValueType::Boolean);
	ResetAction = MakeAction(TEXT("IA_Reset"), EInputActionValueType::Boolean);
	SpawnAction = MakeAction(TEXT("IA_ResetToSpawn"), EInputActionValueType::Boolean);
	AutoShiftAction = MakeAction(TEXT("IA_AutoShift"), EInputActionValueType::Boolean);
	FfbTestAction = MakeAction(TEXT("IA_FfbTest"), EInputActionValueType::Boolean);
	FfbUpAction = MakeAction(TEXT("IA_FfbUp"), EInputActionValueType::Boolean);
	FfbDownAction = MakeAction(TEXT("IA_FfbDown"), EInputActionValueType::Boolean);
	GraphicsAction = MakeAction(TEXT("IA_Graphics"), EInputActionValueType::Boolean);
	PhysicsAction = MakeAction(TEXT("IA_Physics"), EInputActionValueType::Boolean);

	Mapping = NewObject<UInputMappingContext>(this, TEXT("IMC_Drift"));
	// Gamepad (Forza-style layout).
	MapAxis(Mapping, SteerAction, EKeys::Gamepad_LeftX, false, 0.06f);
	MapAxis(Mapping, ThrottleAction, EKeys::Gamepad_RightTriggerAxis);
	MapAxis(Mapping, BrakeAction, EKeys::Gamepad_LeftTriggerAxis);
	MapAxis(Mapping, HandbrakeAction, EKeys::Gamepad_FaceButton_Bottom);   // A
	Mapping->MapKey(ShiftUpAction, EKeys::Gamepad_FaceButton_Right);        // B
	Mapping->MapKey(ShiftDownAction, EKeys::Gamepad_FaceButton_Left);       // X
	Mapping->MapKey(CameraAction, EKeys::Gamepad_FaceButton_Top);           // Y
	Mapping->MapKey(ResetAction, EKeys::Gamepad_Special_Left);              // View / Back
	Mapping->MapKey(AutoShiftAction, EKeys::Gamepad_RightThumbstick);
	// Keyboard.
	MapAxis(Mapping, SteerAction, EKeys::D);
	MapAxis(Mapping, SteerAction, EKeys::A, true);
	MapAxis(Mapping, SteerAction, EKeys::Right);
	MapAxis(Mapping, SteerAction, EKeys::Left, true);
	MapAxis(Mapping, ThrottleAction, EKeys::W);
	MapAxis(Mapping, ThrottleAction, EKeys::Up);
	MapAxis(Mapping, BrakeAction, EKeys::S);
	MapAxis(Mapping, BrakeAction, EKeys::Down);
	MapAxis(Mapping, HandbrakeAction, EKeys::SpaceBar);
	Mapping->MapKey(ShiftUpAction, EKeys::E);
	Mapping->MapKey(ShiftDownAction, EKeys::Q);
	Mapping->MapKey(CameraAction, EKeys::C);
	Mapping->MapKey(ResetAction, EKeys::R);
	Mapping->MapKey(SpawnAction, EKeys::BackSpace);
	Mapping->MapKey(SpawnAction, EKeys::Gamepad_Special_Right);   // Menu / Start
	Mapping->MapKey(AutoShiftAction, EKeys::M);
	Mapping->MapKey(FfbTestAction, EKeys::T);
	Mapping->MapKey(FfbUpAction, EKeys::RightBracket);
	Mapping->MapKey(FfbDownAction, EKeys::LeftBracket);
	Mapping->MapKey(GraphicsAction, EKeys::F5);
	Mapping->MapKey(PhysicsAction, EKeys::F6);
}

void ADriftCarPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	CreateInput();
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		if (UEnhancedInputLocalPlayerSubsystem* Sub = ULocalPlayer::GetSubsystem<UEnhancedInputLocalPlayerSubsystem>(PC->GetLocalPlayer()))
		{
			Sub->AddMappingContext(Mapping, 0);
		}
	}
	if (UEnhancedInputComponent* Eic = Cast<UEnhancedInputComponent>(PlayerInputComponent))
	{
		Eic->BindAction(ShiftUpAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnShiftUp);
		Eic->BindAction(ShiftDownAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnShiftDown);
		Eic->BindAction(CameraAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnToggleCamera);
		Eic->BindAction(ResetAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnReset);
		Eic->BindAction(SpawnAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnResetToSpawn);
		Eic->BindAction(AutoShiftAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnToggleAutoShift);
		Eic->BindAction(FfbTestAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnFfbTest);
		Eic->BindAction(FfbUpAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnFfbStronger);
		Eic->BindAction(FfbDownAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnFfbWeaker);
		Eic->BindAction(GraphicsAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnCycleGraphics);
		Eic->BindAction(PhysicsAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnTogglePhysics);
	}
	else
	{
		UE_LOG(LogDrift, Error, TEXT("Enhanced Input isn't the default input component - check Config/DefaultInput.ini."));
	}
}

void ADriftCarPawn::ReadInput(float DeltaSeconds)
{
	(void)DeltaSeconds;
	drift::Inputs& In = Vehicle->in;
	In.steering_assist = SteeringAssist;
	In.auto_shift = bAutoShift;
	In.auto_clutch = true;
	In.direct_steer = false;
	if (ReadWheel())
	{
		return;
	}
	const APlayerController* PC = Cast<APlayerController>(GetController());
	const UEnhancedPlayerInput* Epi = PC ? Cast<UEnhancedPlayerInput>(PC->PlayerInput) : nullptr;
	if (!Epi || !Mapping)
	{
		In.steer = In.throttle = In.brake = In.handbrake = 0.0;
		return;
	}
	auto Value = [Epi](const UInputAction* A) { return FMath::Clamp(Epi->GetActionValue(A).Get<float>(), -1.f, 1.f); };
	// A touch of expo on the stick so small corrections stay small.
	const float S = Value(SteerAction);
	In.steer = FMath::Sign(S) * FMath::Pow(FMath::Abs(S), 1.4f);
	In.throttle = FMath::Max(0.f, Value(ThrottleAction));
	In.brake = FMath::Max(0.f, Value(BrakeAction));
	In.handbrake = FMath::Max(0.f, Value(HandbrakeAction));
}

bool ADriftCarPawn::ReadWheel()
{
	if (!Wheel)
	{
		return false;
	}
	Wheel->Poll();
	if (!Wheel->HasControls())
	{
		return false;
	}
	const FDriftWheelMapping& M = Wheel->Map;
	drift::Inputs& In = Vehicle->in;
	// Wheel angle -> road-wheel angle through the car's steering ratio.
	const double WheelDeg = Wheel->Axis(M.SteerAxis) * M.RotationDeg * 0.5;
	const double LockDeg = FMath::RadiansToDegrees(Vehicle->Spec().steer_lock);
	In.steer = FMath::Clamp(WheelDeg / SteerRatio / LockDeg, -1.0, 1.0);
	In.direct_steer = true;
	In.steering_assist = 0.0;
	auto Pedal = [&M, this](int32 Axis) {
		if (!Wheel->AxisLive(Axis))
		{
			return 0.f;
		}
		const float V = Wheel->Axis(Axis);
		return FMath::Clamp(M.bPedalsInverted ? (1.f - V) * 0.5f : (V + 1.f) * 0.5f, 0.f, 1.f);
	};
	In.throttle = Pedal(M.ThrottleAxis);
	In.brake = Pedal(M.BrakeAxis);
	In.clutch = Pedal(M.ClutchAxis);
	bClutchUsed |= In.clutch > 0.5;
	In.auto_clutch = !bClutchUsed;
	In.handbrake = Wheel->Button(M.Handbrake) ? 1.0 : 0.0;

	if (Wheel->Pressed(M.ShiftUp)) OnShiftUp();
	if (Wheel->Pressed(M.ShiftDown)) OnShiftDown();
	if (Wheel->Pressed(M.Camera)) OnToggleCamera();
	if (Wheel->Pressed(M.Reset)) OnReset();
	if (Wheel->Pressed(M.ResetToSpawn)) OnResetToSpawn();
	if (Wheel->Pressed(M.FfbUp)) AdjustFfbStrength(1);
	if (Wheel->Pressed(M.FfbDown)) AdjustFfbStrength(-1);
	if (M.FirstGearButton >= 0)
	{
		// H-shifter: out of gear = neutral once it's been used.
		int32 Gear = Wheel->Button(M.ReverseButton) ? -1 : -2;
		for (int32 K = 0; K < 6; ++K)
		{
			if (Wheel->Button(M.FirstGearButton + K))
			{
				Gear = K + 1;
			}
		}
		if (Gear != -2)
		{
			bShifterUsed = true;
			bAutoShift = false;
			// A gate the car doesn't have (6th on a 5-speed) is neutral. The gear goes in through the
			// synchro model (clutch pressed, or rev-matched within the car's shift window).
			Vehicle->SelectGear(Gear > Vehicle->GearCount() ? 0 : Gear);
		}
		else if (bShifterUsed)
		{
			Vehicle->SelectGear(0);
		}
	}
	return true;
}

void ADriftCarPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (!Vehicle || !PhysWorld)
	{
		return;
	}
	if (!IsLocallyControlled())
	{
		if (bLocalStarted)
		{
			UE_LOG(LogDriftNet, Warning, TEXT("%s was local, now controller=%s"), *GetName(), Controller ? *Controller->GetName() : TEXT("none"));
			bLocalStarted = false;
		}
		ApplyRemote(DeltaSeconds);
		return;
	}
	if (!bLocalStarted)
	{
		StartLocal();
	}
	ReadInput(DeltaSeconds);
	if (CVarAutoThrottle.GetValueOnGameThread() >= 0.f)
	{
		Vehicle->in.throttle = CVarAutoThrottle.GetValueOnGameThread();
		Vehicle->in.brake = 0.0;
		Vehicle->in.auto_clutch = true;
	}
	if (CVarAutoSteer.GetValueOnGameThread() >= -1.f)
	{
		Vehicle->in.steer = CVarAutoSteer.GetValueOnGameThread();
		Vehicle->in.direct_steer = true;
	}
	{
		const double Played = GetWorld()->GetTimeSeconds();
		const float Shot = CVarAutoShot.GetValueOnGameThread(), Quit = CVarAutoQuit.GetValueOnGameThread();
		if (Shot > 0.f && Played >= Shot && !bAutoShotDone)
		{
			bAutoShotDone = true;
			if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
			{
				PC->ConsoleCommand(TEXT("HighResShot 1600x900"));   // a viewport command, not an engine one
			}
		}
		if (Quit > 0.f && Played >= Quit)
		{
			FPlatformMisc::RequestExit(false);
		}
	}
	if (CVarPhysicsLog.GetValueOnGameThread() && FPlatformTime::Seconds() >= NextPhysicsLog)
	{
		NextPhysicsLog = FPlatformTime::Seconds() + 1.0;
		const drift::Vec3 O = Vehicle->Origin();
		UE_LOG(LogDrift, Log, TEXT("[phys %s %.2f ms dt %.3f] origin (%.2f %.2f %.2f) %.1f km/h gear %d rpm %.0f drift %.1f wheels on road %d ffb %+.2f fz %.0f %.0f %.0f %.0f"),
			*GetPhysicsName(), GetPhysicsMs(), DeltaSeconds, O.x, O.y, O.z, Vehicle->Speed() * 3.6, Vehicle->Gear(), Vehicle->Rpm(), Vehicle->DriftAngleDeg(),
			Vehicle->OnRoadWheels(), Vehicle->FfbForce(), Vehicle->Wheel(0).fz, Vehicle->Wheel(1).fz, Vehicle->Wheel(2).fz, Vehicle->Wheel(3).fz);
	}
	if (Chrono)
	{
		Chrono->Step(DeltaSeconds, *Vehicle, *PhysWorld);
	}
	else
	{
		Vehicle->Step(DeltaSeconds, *PhysWorld);
	}
	if (Vehicle->OnRoadWheels() >= 3)
	{
		LastOnRoad = GetActorLocation();
		LastOnRoadYaw = GetActorRotation().Yaw;
	}
	// Fell off the world: put it back where it was last on the road.
	// (or more than 8 m below where it was last on the road: it went through the ground - don't
	// make the driver watch a 300 m fall).
	if (Vehicle->Origin().y < LowestY || Vehicle->Origin().y < LastOnRoad.Z * 0.01 - 8.0)
	{
		ResetCar(LastOnRoad, LastOnRoadYaw);
	}
	SetActorLocationAndRotation(DriftUE::Pos(Vehicle->Origin()), DriftUE::Rot(Vehicle->Rotation()));
	SyncVisuals(DeltaSeconds);
	if (Telemetry)
	{
		Telemetry->Sample(DeltaSeconds, *Vehicle, GetActorLocation());
	}
	SendNetState();
	if (Wheel)
	{
		Vehicle->ffb_fx.road = Wheel->Map.FfbRoad;
		Vehicle->ffb_fx.kerb = Wheel->Map.FfbKerb;
		Vehicle->ffb_fx.slip = Wheel->Map.FfbSlip;
		Wheel->SendForce(static_cast<float>(Vehicle->FfbForce()), static_cast<float>(Vehicle->FfbDamper()), static_cast<float>(Vehicle->FfbSpring()),
			static_cast<float>(Vehicle->FfbVibAmp()), static_cast<float>(Vehicle->FfbVibHz()));
	}
}

void ADriftCarPawn::SyncVisuals(float DeltaSeconds)
{
	// Unreal's FOV is horizontal; AC's (and our setting) vertical - convert for
	// the current screen shape so the cockpit looks the same size as in AC.
	if (GEngine && GEngine->GameViewport && GEngine->GameViewport->Viewport)
	{
		const FIntPoint Size = GEngine->GameViewport->Viewport->GetSizeXY();
		const float Aspect = Size.Y > 0 ? static_cast<float>(Size.X) / Size.Y : 16.f / 9.f;
		if (!FMath::IsNearlyEqual(Aspect, LastAspect, 0.001f))
		{
			LastAspect = Aspect;
			auto HFov = [Aspect](float VDeg) { return FMath::RadiansToDegrees(2.f * FMath::Atan(FMath::Tan(FMath::DegreesToRadians(VDeg) * 0.5f) * Aspect)); };
			CockpitCamera->SetFieldOfView(HFov(CockpitVFov));
			HoodCamera->SetFieldOfView(HFov(HoodVFov));
		}
	}
	for (int32 I = 0; I < 4; ++I)
	{
		const drift::WheelState& W = Vehicle->Wheel(I);
		WheelHubs[I]->SetRelativeLocation(DriftUE::Pos(Vehicle->WheelCenterLocal(I)));
		// Hub steers (Unreal yaw + = turn right, same as DriftCore), the wheel
		// spins inside it (rolling forward is negative pitch).
		WheelHubs[I]->SetRelativeRotation(FRotator(0.0, FMath::RadiansToDegrees(W.steer), 0.0));
		WheelMeshes[I]->SetRelativeRotation(FRotator(-FMath::RadiansToDegrees(W.spin), 0.0, 0.0).Quaternion() * WheelMeshBase);
	}
	if (bHasModel)
	{
		// Same as Drift Lab: turn about the column by steer x ratio (sign checked
		// in the cockpit: steer left turns the rim left).
		const double Steer = (Vehicle->Wheel(0).steer + Vehicle->Wheel(1).steer) * 0.5;
		SteerMesh->SetRelativeRotation(FQuat(SteerAxis, Steer * SteerRatio) * ModelBase);
	}
	// Chase camera: swings toward the direction of travel, so you see where
	// the car is going while it's sideways.
	const FRotator CarRot = GetActorRotation();
	float TargetYaw = CarRot.Yaw;
	const FVector Vel = DriftUE::Dir(Vehicle->Velocity());
	const double Speed = Vel.Size2D();
	if (Speed > 400.0)
	{
		const float VelYaw = Vel.Rotation().Yaw;
		const float Blend = FMath::Clamp(static_cast<float>((Speed - 400.0) / 800.0), 0.f, 1.f) * 0.55f;
		TargetYaw = CarRot.Yaw + FMath::FindDeltaAngleDegrees(CarRot.Yaw, VelYaw) * Blend;
	}
	const FRotator Target(-9.f, TargetYaw, 0.f);
	ChaseRotation = FMath::RInterpTo(ChaseRotation, Target, DeltaSeconds, 5.f);
	ChaseArm->SetWorldRotation(ChaseRotation);
}

void ADriftCarPawn::OnShiftUp()
{
	if (Vehicle)
	{
		Vehicle->ShiftUp();
	}
}

void ADriftCarPawn::OnShiftDown()
{
	if (Vehicle)
	{
		Vehicle->ShiftDown();
	}
}

void ADriftCarPawn::OnToggleCamera()
{
	SetCamera((CameraMode + 1) % 3);
}

void ADriftCarPawn::SetCamera(int32 Mode)
{
	CameraMode = Mode;
	const bool bCockpit = Mode == 1;
	ChaseCamera->SetActive(Mode == 0);
	CockpitCamera->SetActive(bCockpit);
	HoodCamera->SetActive(Mode == 2);
	// The placeholder body is a solid box - hide it from inside. A real model
	// has its interior.
	Body->SetVisibility(!bCockpit || bHasModel);
}

void ADriftCarPawn::OnReset()
{
	ResetCar(GetActorLocation(), GetActorRotation().Yaw);
}

void ADriftCarPawn::OnResetToSpawn()
{
	ResetCar(SpawnLocation, SpawnYaw);
	LastOnRoad = SpawnLocation;
	LastOnRoadYaw = SpawnYaw;
}

void ADriftCarPawn::OnToggleAutoShift()
{
	bAutoShift = !bAutoShift;
}

void ADriftCarPawn::OnCycleGraphics()
{
	GraphicsPreset = (GraphicsPreset + 1) % DriftGraphics::Count;
	DriftGraphics::Apply(GraphicsPreset);
	GraphicsShownUntil = FPlatformTime::Seconds() + 3.0;
}

void ADriftCarPawn::AdjustFfbStrength(int32 Dir)
{
	if (!Chrono || Chrono->SteerTorqueRef <= 0.0)
	{
		return;
	}
	// Shown as a percentage of the configured reference (DefaultGame.ini FfbSteerTorque = 100 %); 10 % steps.
	const double Base = Chrono->DefaultSteerTorqueRef * 100.0;
	const double Pct = FMath::Clamp(FMath::RoundToDouble(Base / Chrono->SteerTorqueRef / 10.0) * 10.0 + Dir * 10.0, 30.0, 250.0);
	Chrono->SteerTorqueRef = Base / Pct;
	GConfig->SetDouble(TEXT("DriftChrono"), TEXT("FfbSteerTorque"), Chrono->SteerTorqueRef, GGameIni);
	GConfig->Flush(false, GGameIni);
	FfbMessage = FString::Printf(TEXT("Wheel force %d%%  ( [ / ] )"), FMath::RoundToInt(Pct));
	FfbShownUntil = FPlatformTime::Seconds() + 2.5;
}

void ADriftCarPawn::OnFfbTest()
{
	if (Wheel)
	{
		Wheel->RunTest();   // pulls left, then right
	}
}

// --- Multiplayer -------------------------------------------------------------

void ADriftCarPawn::GetLifetimeReplicatedProps(TArray<FLifetimeProperty>& OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME_CONDITION(ADriftCarPawn, NetState, COND_SkipOwner);
}

void ADriftCarPawn::SendNetState()
{
	if (GetNetMode() == NM_Standalone)
	{
		return;
	}
	const double Now = FPlatformTime::Seconds();
	if (Now < NextNetSend)
	{
		return;
	}
	NextNetSend = Now + 1.0 / 30.0;
	if (Now > NextNetLog)
	{
		NextNetLog = Now + 2.0;
		UE_LOG(LogDriftNet, Verbose, TEXT("local %s at %s"), *GetName(), *GetActorLocation().ToCompactString());
	}
	FDriftNetState S;
	S.Location = GetActorLocation();
	S.Rotation = GetActorRotation();
	S.Velocity = DriftUE::Dir(Vehicle->Velocity()) * 100.0;
	S.Steer = FMath::RadiansToDegrees((Vehicle->Wheel(0).steer + Vehicle->Wheel(1).steer) * 0.5);
	for (int32 I = 0; I < 4; ++I)
	{
		S.WheelOmega[I] = static_cast<float>(Vehicle->Wheel(I).omega);
		S.Compression[I] = static_cast<float>(Vehicle->Wheel(I).compression);
	}
	S.Rpm = static_cast<float>(Vehicle->Rpm());
	S.Gear = static_cast<int8>(Vehicle->Gear());
	S.Time = static_cast<float>(Now);
	if (HasAuthority())
	{
		NetState = S;   // host's own car: replicates to everyone else
	}
	else
	{
		ServerSetState(S);
	}
}

void ADriftCarPawn::ServerSetState_Implementation(const FDriftNetState& State)
{
	NetState = State;        // on to the other clients
	PushSnapshot(State);     // and shown on the host
}

void ADriftCarPawn::OnRep_NetState()
{
	PushSnapshot(NetState);
}

void ADriftCarPawn::PushSnapshot(const FDriftNetState& State)
{
	// Map the owner's clock onto ours with the smallest offset seen (the least
	// delayed packet), so playback runs at their pace, not packet arrival.
	const double Now = FPlatformTime::Seconds();
	const double Offset = Now - State.Time;
	if (!bHaveClock || Offset < RemoteClockOffset)
	{
		RemoteClockOffset = Offset;
		bHaveClock = true;
	}
	else
	{
		RemoteClockOffset += (Offset - RemoteClockOffset) * 0.01;   // follow clock drift slowly
	}
	if (Snaps.Num() == 0)
	{
		UE_LOG(LogDriftNet, Log, TEXT("Receiving %s's car"), *GetName());
	}
	if (Snaps.Num() > 0 && State.Time <= Snaps.Last().S.Time)
	{
		return;   // out of order
	}
	Snaps.Add({State.Time + RemoteClockOffset, State});
	if (Snaps.Num() > 30)
	{
		Snaps.RemoveAt(0, Snaps.Num() - 30);
	}
}

void ADriftCarPawn::ApplyRemote(float DeltaSeconds)
{
	if (Snaps.Num() == 0 || !Vehicle)
	{
		return;
	}
	// Show other cars 100 ms in the past, between two received states; if the
	// next one is late, carry on along the last velocity for up to 250 ms.
	const double RenderAt = FPlatformTime::Seconds() - 0.1;
	const FDriftNetState* A = &Snaps[0].S;
	const FDriftNetState* B = nullptr;
	double Alpha = 0.0;
	for (int32 I = 0; I + 1 < Snaps.Num(); ++I)
	{
		if (Snaps[I].Arrived <= RenderAt && RenderAt <= Snaps[I + 1].Arrived)
		{
			A = &Snaps[I].S;
			B = &Snaps[I + 1].S;
			Alpha = (RenderAt - Snaps[I].Arrived) / FMath::Max(1e-4, Snaps[I + 1].Arrived - Snaps[I].Arrived);
			break;
		}
		A = &Snaps[I + 1].S;
	}
	FVector Loc;
	FQuat Rot;
	float Steer;
	if (B)
	{
		Loc = FMath::Lerp(FVector(A->Location), FVector(B->Location), Alpha);
		Rot = FQuat::Slerp(A->Rotation.Quaternion(), B->Rotation.Quaternion(), Alpha);
		Steer = FMath::Lerp(A->Steer, B->Steer, static_cast<float>(Alpha));
	}
	else
	{
		const double Ahead = FMath::Clamp(RenderAt - Snaps.Last().Arrived, 0.0, 0.25);
		A = &Snaps.Last().S;
		Loc = FVector(A->Location) + FVector(A->Velocity) * Ahead;
		Rot = A->Rotation.Quaternion();
		Steer = A->Steer;
	}
	SetActorLocationAndRotation(Loc, Rot);
	RemoteVelocity = FVector(B ? FMath::Lerp(FVector(A->Velocity), FVector(B->Velocity), Alpha) : FVector(A->Velocity));
	if (FPlatformTime::Seconds() > NextNetSend)   // reused as a log timer on remote cars
	{
		NextNetSend = FPlatformTime::Seconds() + 2.0;
		UE_LOG(LogDriftNet, Verbose, TEXT("remote %s at %s"), *GetName(), *Loc.ToCompactString());
	}
	for (int32 I = 0; I < 4; ++I)
	{
		const drift::WheelState& W = Vehicle->Wheel(I);
		RemoteSpin[I] += A->WheelOmega[I] * DeltaSeconds;
		const drift::Vec3 Centre = W.mount - drift::Vec3(0, W.travel - A->Compression[I], 0);
		WheelHubs[I]->SetRelativeLocation(DriftUE::Pos(Centre));
		WheelHubs[I]->SetRelativeRotation(FRotator(0.0, W.front ? Steer : 0.0, 0.0));
		WheelMeshes[I]->SetRelativeRotation(FRotator(-FMath::RadiansToDegrees(RemoteSpin[I]), 0.0, 0.0).Quaternion() * WheelMeshBase);
	}
	if (bHasModel)
	{
		SteerMesh->SetRelativeRotation(FQuat(SteerAxis, FMath::DegreesToRadians(Steer) * SteerRatio) * ModelBase);
	}
}

FVector ADriftCarPawn::GetCarVelocity() const
{
	if (Vehicle && IsLocallyControlled())
	{
		return DriftUE::Dir(Vehicle->Velocity()) * 100.0;
	}
	return RemoteVelocity;
}

double ADriftCarPawn::GetCarMass() const
{
	return Vehicle ? Vehicle->Spec().mass : 1200.0;
}
