#include "DriftCarPawn.h"

#include <string>

#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "EnhancedPlayerInput.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "UObject/ConstructorHelpers.h"
#include "drift/CarSpec.h"
#include "drift/Json.h"
#include "DriftUE.h"

DEFINE_LOG_CATEGORY_STATIC(LogDrift, Log, All);

ADriftCarPawn::ADriftCarPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	AutoPossessPlayer = EAutoReceiveInput::Player0;

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
	ResetCar(GetActorLocation(), GetActorRotation().Yaw);
	LastOnRoad = GetActorLocation();
	LastOnRoadYaw = GetActorRotation().Yaw;
	LowestY = Vehicle->Origin().y - 300.0;
	ChaseRotation = FRotator(-9.f, GetActorRotation().Yaw, 0.f);
	Wheel = std::make_unique<FDriftWheel>();
	Wheel->Start();
}

void ADriftCarPawn::EndPlay(const EEndPlayReason::Type Reason)
{
	Wheel.reset();   // stops the helper: the wheel goes limp
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
	SetActorLocationAndRotation(DriftUE::Pos(Vehicle->Origin()), DriftUE::Rot(Vehicle->Rotation()));
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
	AutoShiftAction = MakeAction(TEXT("IA_AutoShift"), EInputActionValueType::Boolean);
	FfbTestAction = MakeAction(TEXT("IA_FfbTest"), EInputActionValueType::Boolean);

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
	Mapping->MapKey(AutoShiftAction, EKeys::M);
	Mapping->MapKey(FfbTestAction, EKeys::T);
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
		Eic->BindAction(AutoShiftAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnToggleAutoShift);
		Eic->BindAction(FfbTestAction, ETriggerEvent::Started, this, &ADriftCarPawn::OnFfbTest);
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
			Vehicle->SetGear(Gear);
		}
		else if (bShifterUsed)
		{
			Vehicle->SetGear(0);
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
	ReadInput(DeltaSeconds);
	Vehicle->Step(DeltaSeconds, *PhysWorld);
	if (Vehicle->OnRoadWheels() >= 3)
	{
		LastOnRoad = GetActorLocation();
		LastOnRoadYaw = GetActorRotation().Yaw;
	}
	// Fell off the world: put it back where it was last on the road.
	if (Vehicle->Origin().y < LowestY)
	{
		ResetCar(LastOnRoad, LastOnRoadYaw);
	}
	SetActorLocationAndRotation(DriftUE::Pos(Vehicle->Origin()), DriftUE::Rot(Vehicle->Rotation()));
	SyncVisuals(DeltaSeconds);
	if (Wheel)
	{
		Wheel->SendForce(static_cast<float>(Vehicle->FfbForce()), static_cast<float>(Vehicle->FfbDamper()), static_cast<float>(Vehicle->FfbSpring()));
	}
}

void ADriftCarPawn::SyncVisuals(float DeltaSeconds)
{
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

void ADriftCarPawn::OnToggleAutoShift()
{
	bAutoShift = !bAutoShift;
}

void ADriftCarPawn::OnFfbTest()
{
	if (Wheel)
	{
		Wheel->RunTest();   // pulls left, then right
	}
}
