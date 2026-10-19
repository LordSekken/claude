// Procedural car sound: engine (inline-4 pulse/additive model, intake, exhaust
// rasp, limiter cut, overrun crackle, turbo whistle, blow-off) and tyres
// (scrub/squeal per wheel, off-road rumble). No samples: the game thread hands
// UDriftEngineSynth a small FDriftAudioParams every frame (lock-free triple
// buffer) and the audio render thread synthesizes it in OnGenerateAudio.
// Gains come from [DriftAudio] in DefaultGame.ini (see DriftAudio_INTEGRATION.md).
#pragma once

#include <atomic>

#include "CoreMinimal.h"
#include "Components/SynthComponent.h"
#include "DriftAudio.generated.h"

namespace drift { class Vehicle; }

/** One tyre, as the sound needs it. */
struct FDriftAudioWheel
{
	float SlipRatio = 0.f;
	float SlipAngle = 0.f;     // rad
	float Load = 0.f;          // N; 0 = in the air
	bool bOnRoad = true;       // false: gravel/grass/dirt/sand -> rumble instead of squeal
	float Pan = 0.f;           // -1 left .. +1 right
};

/** What the synth needs from the car, sent once a game frame. */
struct FDriftAudioParams
{
	float Rpm = 0.f;
	float Throttle = 0.f;      // 0..1
	float Load = 0.f;          // -1 (engine braking) .. 1 (full torque)
	int32 Gear = 0;            // -1 R, 0 N, 1..
	bool bLimiter = false;     // rev limiter cutting fuel
	float BoostBar = 0.f;      // turbo boost, bar (gauge)
	float SpeedMs = 0.f;
	FDriftAudioWheel Wheels[4];

	/** The local car, straight from DriftCore (also valid on the Chrono backend). */
	static FDriftAudioParams FromVehicle(const drift::Vehicle& V);
	/** Other players' cars: only rpm/gear/speed are replicated, so throttle is a guess and the tyres are quiet. */
	static FDriftAudioParams FromRemote(float InRpm, int32 InGear, float InSpeedMs);
};

/** [DriftAudio] in DefaultGame.ini. Gains are linear (1 = default level, 0 = off). */
struct FDriftAudioConfig
{
	bool bEnabled = true;
	float MasterGain = 0.8f;
	float EngineGain = 1.f;
	float IntakeGain = 0.6f;
	float ExhaustNoiseGain = 0.4f;
	float TurboGain = 0.3f;
	float BlowOffGain = 0.5f;
	float CrackleGain = 0.4f;
	float GearWhineGain = 0.f;
	float TyreGain = 0.8f;
	float SquealGain = 1.f;
	float ScrubGain = 0.6f;
	float OffRoadGain = 0.7f;
	float RoadGain = 0.25f;
	int32 Cylinders = 4;               // even-firing 4-stroke
	float LimiterHz = 16.f;            // fuel-cut bounce rate on the limiter
	float ExhaustResonanceHz = 140.f;  // muffler/tailpipe body
	float ExhaustResonance2Hz = 480.f; // pipe "honk"
	float TurboMinHz = 1800.f;         // whistle pitch just spooling
	float TurboMaxHz = 7500.f;         // ... at TurboRefBar
	float TurboRefBar = 1.f;
	float BlowOffMinBar = 0.3f;        // boost needed for a blow-off on throttle lift
	float TyreSlipThreshold = 0.08f;   // combined slip where squeal starts
	float TyreSlipFull = 0.35f;        // ... and is at full level
	float TyreRefLoad = 3500.f;        // N; a tyre at this load squeals at full level
	float SquealHz = 950.f;
	float TyreStereoWidth = 0.7f;      // 0 = mono tyres, 1 = hard left/right

	/** Game thread. Missing keys keep the defaults above. */
	void Load();
};

/** Cytomic (Simper) trapezoidal state-variable filter: stable under per-block retuning. */
struct FDriftSvf
{
	float IC1 = 0.f, IC2 = 0.f;
	float A1 = 1.f, A2 = 0.f, A3 = 0.f, K = 1.f;

	void Set(float Hz, float Q, float InvRate)
	{
		const float G = FMath::Tan(UE_PI * FMath::Clamp(Hz * InvRate, 1e-5f, 0.45f));
		K = 1.f / FMath::Max(Q, 0.05f);
		A1 = 1.f / (1.f + G * (G + K));
		A2 = G * A1;
		A3 = G * A2;
	}
	void CopyTuning(const FDriftSvf& O) { A1 = O.A1; A2 = O.A2; A3 = O.A3; K = O.K; }
	FORCEINLINE void Tick(float In, float& Lp, float& Bp)
	{
		const float V3 = In + 1e-20f - IC2;   // tiny offset: states never go denormal
		const float V1 = A1 * IC1 + A2 * V3;
		const float V2 = IC2 + A2 * IC1 + A3 * V3;
		IC1 = 2.f * V1 - IC1;
		IC2 = 2.f * V2 - IC2;
		Lp = V2;
		Bp = V1;
	}
	FORCEINLINE float Low(float In) { float L, B; Tick(In, L, B); return L; }
	/** Band-pass with unity gain at the centre frequency. */
	FORCEINLINE float Band(float In) { float L, B; Tick(In, L, B); return B * K; }
	FORCEINLINE float High(float In) { float L, B; Tick(In, L, B); return In - K * B - L; }
};

/** xorshift32 white noise, -1..1. */
struct FDriftNoise
{
	uint32 S = 0x9E3779B9u;
	FORCEINLINE float Next()
	{
		S ^= S << 13;
		S ^= S >> 17;
		S ^= S << 5;
		return static_cast<float>(static_cast<int32>(S)) * (1.f / 2147483648.f);
	}
	FORCEINLINE float Next01() { return 0.5f * (Next() + 1.f); }
};

/** One-pole smoother, run per sample so parameter steps never zipper. */
struct FDriftSmooth
{
	float V = 0.f, A = 1.f;
	void Init(float Seconds, float Rate) { A = 1.f - FMath::Exp(-1.f / (Seconds * Rate)); }
	FORCEINLINE float Tick(float Target) { V += A * (Target - V) + 1e-25f; return V; }
};

/** The DSP itself: plain C++, owned by UDriftEngineSynth, used only on the audio render thread. */
class FDriftAudioRenderer
{
public:
	void Init(int32 InSampleRate);
	/** Interleaved output, Frames x Channels. */
	void Render(const FDriftAudioParams& P, const FDriftAudioConfig& C, float* Out, int32 Frames, int32 Channels);

private:
	static constexpr int32 kBlock = 32;     // control rate: filter retuning every 32 samples
	static constexpr int32 kMaxHarm = 24;

	struct FWheelVoice
	{
		FDriftSvf SquealBp, ScrubBp, RumbleLp, CrunchBp;
		FDriftSmooth Squeal, Scrub, Rumble, Pitch, Vibrato, Grain;
		FDriftNoise Noise, Noise2;
		float Phase = 0.f;
		float Detune = 1.f;
		float GainL = 1.f, GainR = 1.f;
		float SquealT = 0.f, ScrubT = 0.f, RumbleT = 0.f, PitchT = 1.f, VibT = 0.f, GrainT = 0.f;
		float ToneMix = 1.f;
		bool bActive = false;
	};

	void BlockControl(const FDriftAudioParams& P, const FDriftAudioConfig& C, int32 Cyl);
	void NewFiring(int32 Slot, const FDriftAudioConfig& C);
	float EngineSample(const FDriftAudioParams& P, const FDriftAudioConfig& C, int32 Cyl);
	void TyreSample(const FDriftAudioConfig& C, float& L, float& R);

	float Rate = 48000.f, InvRate = 1.f / 48000.f;
	uint32 BlockCount = 0;

	// Engine state
	FDriftSmooth RpmS, ThrS, LoadS, BoostS, ThrFastS, ThrSlowS, LimEnvS, BovAtkS, TurboLvlS, WhineS, RoadS;
	FDriftSvf ExDc, ExRes1, ExRes2, ExLp1, ExLp2, IntakeBp, RaspBp, PopBp, TurboBp, BovBp, RoadLp;
	FDriftNoise Noise, Noise2, PopNoise, FireNoise;
	float CylGain[12];
	float Harm[kMaxHarm + 1];
	int32 NumHarm = 8;
	float CyclePhase = 0.f;        // 0..1 over one 4-stroke cycle (two crank turns)
	int32 LastSlot = -1;
	float FireAmp = 0.f, FireDelay = 0.f, FireInvWidth = 2.f;
	float LimPhase = 0.f;
	bool bLimiterWas = false, bCutting = false;
	float PopEnv = 0.f, PopDecay = 0.f;
	float OverrunTime = 0.f;
	float BovEnv = 0.f, BovDecay = 0.f;
	bool bBovArmed = true;
	float TurboPhase = 0.f, TurboHz = 2000.f, TurboLvlT = 0.f;
	float WhinePhase = 0.f, WhineT = 0.f;
	float RoadT = 0.f;

	FWheelVoice Wheels[4];
};

/**
 * Engine + tyre sound of one car. Add as a component, call StartSound once the
 * car knows whether it is the local player's, then SetParams every game frame.
 */
UCLASS(ClassGroup = Drift, meta = (BlueprintSpawnableComponent))
class DRIFTGAME_API UDriftEngineSynth : public USynthComponent
{
	GENERATED_BODY()

public:
	UDriftEngineSynth(const FObjectInitializer& ObjectInitializer);

	/**
	 * Game thread. Reads [DriftAudio] and starts playing (once; later calls just
	 * return whether it plays). Local car: 2D stereo (tyres panned by side).
	 * Remote cars: spatialized at this component with distance falloff.
	 * False if disabled in config or there is no audio device (server, -nosound).
	 */
	bool StartSound(bool bLocalCar);

	/** Game thread, every frame. Never blocks; the audio thread takes the newest. */
	void SetParams(const FDriftAudioParams& P);

	/** Game thread: re-read [DriftAudio] and hand it to the audio thread. */
	void ReloadConfig();

protected:
	virtual bool Init(int32& SampleRate) override;
	virtual int32 OnGenerateAudio(float* OutAudio, int32 NumSamples) override;

private:
	// Triple buffer: game thread writes Slots[WriteSlot], audio thread reads
	// Slots[ReadSlot], Middle swaps between them (bit 2 = fresh data waiting).
	FDriftAudioParams Slots[3];
	std::atomic<uint32> Middle{1};
	uint32 WriteSlot = 0;          // game thread only
	uint32 ReadSlot = 2;           // audio thread only

	FDriftAudioConfig Cfg;         // audio thread after Start (changed via SynthCommand)
	FDriftAudioRenderer Renderer;  // audio thread
	bool bStartTried = false;
};
