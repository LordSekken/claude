#include "DriftAudio.h"

#include "Misc/ConfigCacheIni.h"
#include "drift/CarSpec.h"
#include "drift/Vehicle.h"

DEFINE_LOG_CATEGORY_STATIC(LogDriftAudio, Log, All);

namespace
{
	constexpr uint32 kFresh = 4u;          // triple buffer: new params waiting
	constexpr float kRpmRef = 8000.f;      // rpm that counts as "high" for levels/brightness
	constexpr float kLimiterDuty = 0.45f;  // fraction of each limiter period with the fuel cut
	constexpr float kWheelDetune[4] = {1.f, 1.035f, 0.96f, 0.99f};   // tyres never squeal in unison

	FORCEINLINE float SoftClip(float X)
	{
		X = FMath::Clamp(X, -3.f, 3.f);
		return X * (27.f + X * X) / (27.f + 9.f * X * X);
	}

	FORCEINLINE float Wrap01(float X) { return X >= 1.f ? X - 1.f : X; }
}

// --- Params / config ---------------------------------------------------------------

FDriftAudioParams FDriftAudioParams::FromVehicle(const drift::Vehicle& V)
{
	FDriftAudioParams P;
	const double CarRpm = V.Rpm();
	const double Thr = FMath::Clamp(V.in.throttle, 0.0, 1.0);
	P.Rpm = static_cast<float>(CarRpm);
	P.Throttle = static_cast<float>(Thr);
	// Load = engine torque now / full-throttle torque at this rpm: negative on the overrun.
	const double Full = V.EngineTorque(CarRpm, 1.0);
	P.Load = Full > 1.0 ? static_cast<float>(FMath::Clamp(V.EngineTorque(CarRpm, Thr) / Full, -1.0, 1.0)) : 0.f;
	P.Gear = V.Gear();
	P.bLimiter = Thr > 0.3 && CarRpm >= V.Spec().limiter_rpm * 0.985;
	P.BoostBar = static_cast<float>(FMath::Max(0.0, V.Boost()));
	P.SpeedMs = static_cast<float>(V.Speed());
	for (int32 I = 0; I < 4; ++I)
	{
		const drift::WheelState& W = V.Wheel(I);
		FDriftAudioWheel& A = P.Wheels[I];
		A.SlipRatio = static_cast<float>(W.slip_ratio);
		A.SlipAngle = static_cast<float>(W.slip_angle);
		A.Load = W.contact ? static_cast<float>(FMath::Max(0.0, W.fz)) : 0.f;
		A.bOnRoad = drift::GetSurfaceInfo(W.surface).road;
		A.Pan = W.left ? -1.f : 1.f;
	}
	return P;
}

FDriftAudioParams FDriftAudioParams::FromRemote(float InRpm, int32 InGear, float InSpeedMs)
{
	FDriftAudioParams P;
	P.Rpm = InRpm;
	P.Gear = InGear;
	P.SpeedMs = InSpeedMs;
	// Off idle a remote car is most likely on the gas (drifting); at idle it is coasting.
	const float OnGas = FMath::SmoothStep(1200.f, 2500.f, InRpm);
	P.Throttle = 0.7f * OnGas;
	P.Load = FMath::Lerp(-0.1f, 0.6f, OnGas);
	for (int32 I = 0; I < 4; ++I)
	{
		P.Wheels[I].Pan = (I % 2 == 0) ? -1.f : 1.f;   // FL, FR, RL, RR; Load 0 = quiet
	}
	return P;
}

void FDriftAudioConfig::Load()
{
	if (!GConfig)
	{
		return;
	}
	const TCHAR* S = TEXT("DriftAudio");
	GConfig->GetBool(S, TEXT("Enabled"), bEnabled, GGameIni);
	GConfig->GetFloat(S, TEXT("MasterGain"), MasterGain, GGameIni);
	GConfig->GetFloat(S, TEXT("EngineGain"), EngineGain, GGameIni);
	GConfig->GetFloat(S, TEXT("IntakeGain"), IntakeGain, GGameIni);
	GConfig->GetFloat(S, TEXT("ExhaustNoiseGain"), ExhaustNoiseGain, GGameIni);
	GConfig->GetFloat(S, TEXT("TurboGain"), TurboGain, GGameIni);
	GConfig->GetFloat(S, TEXT("BlowOffGain"), BlowOffGain, GGameIni);
	GConfig->GetFloat(S, TEXT("CrackleGain"), CrackleGain, GGameIni);
	GConfig->GetFloat(S, TEXT("GearWhineGain"), GearWhineGain, GGameIni);
	GConfig->GetFloat(S, TEXT("TyreGain"), TyreGain, GGameIni);
	GConfig->GetFloat(S, TEXT("SquealGain"), SquealGain, GGameIni);
	GConfig->GetFloat(S, TEXT("ScrubGain"), ScrubGain, GGameIni);
	GConfig->GetFloat(S, TEXT("OffRoadGain"), OffRoadGain, GGameIni);
	GConfig->GetFloat(S, TEXT("RoadGain"), RoadGain, GGameIni);
	GConfig->GetInt(S, TEXT("Cylinders"), Cylinders, GGameIni);
	GConfig->GetFloat(S, TEXT("LimiterHz"), LimiterHz, GGameIni);
	GConfig->GetFloat(S, TEXT("ExhaustResonanceHz"), ExhaustResonanceHz, GGameIni);
	GConfig->GetFloat(S, TEXT("ExhaustResonance2Hz"), ExhaustResonance2Hz, GGameIni);
	GConfig->GetFloat(S, TEXT("TurboMinHz"), TurboMinHz, GGameIni);
	GConfig->GetFloat(S, TEXT("TurboMaxHz"), TurboMaxHz, GGameIni);
	GConfig->GetFloat(S, TEXT("TurboRefBar"), TurboRefBar, GGameIni);
	GConfig->GetFloat(S, TEXT("BlowOffMinBar"), BlowOffMinBar, GGameIni);
	GConfig->GetFloat(S, TEXT("TyreSlipThreshold"), TyreSlipThreshold, GGameIni);
	GConfig->GetFloat(S, TEXT("TyreSlipFull"), TyreSlipFull, GGameIni);
	GConfig->GetFloat(S, TEXT("TyreRefLoad"), TyreRefLoad, GGameIni);
	GConfig->GetFloat(S, TEXT("SquealHz"), SquealHz, GGameIni);
	GConfig->GetFloat(S, TEXT("TyreStereoWidth"), TyreStereoWidth, GGameIni);

	// Keep the DSP in a sane range whatever the ini says.
	Cylinders = FMath::Clamp(Cylinders, 1, 12);
	LimiterHz = FMath::Clamp(LimiterHz, 2.f, 40.f);
	TurboRefBar = FMath::Max(TurboRefBar, 0.1f);
	TyreSlipThreshold = FMath::Clamp(TyreSlipThreshold, 0.f, 1.f);
	TyreSlipFull = FMath::Max(TyreSlipFull, TyreSlipThreshold + 0.01f);
	TyreRefLoad = FMath::Max(TyreRefLoad, 100.f);
	TyreStereoWidth = FMath::Clamp(TyreStereoWidth, 0.f, 1.f);
}

// --- Renderer (audio render thread) ----------------------------------------------

void FDriftAudioRenderer::Init(int32 InSampleRate)
{
	Rate = static_cast<float>(FMath::Max(InSampleRate, 8000));
	InvRate = 1.f / Rate;

	RpmS.Init(0.025f, Rate);
	ThrS.Init(0.03f, Rate);
	LoadS.Init(0.05f, Rate);
	BoostS.Init(0.05f, Rate);
	ThrFastS.Init(0.012f, Rate);
	ThrSlowS.Init(0.15f, Rate);
	LimEnvS.Init(0.003f, Rate);
	BovAtkS.Init(0.004f, Rate);
	TurboLvlS.Init(0.03f, Rate);
	WhineS.Init(0.05f, Rate);
	RoadS.Init(0.1f, Rate);
	PopDecay = FMath::Exp(-1.f / (0.012f * Rate));
	BovDecay = FMath::Exp(-1.f / (0.22f * Rate));

	Noise.S = 0x12345678u;
	Noise2.S = 0x2468ACE1u;
	PopNoise.S = 0x0BADF00Du;
	FireNoise.S = 0x31415927u;
	// Each cylinder is a little different (flow, mixture): gives the 0.5-order lope.
	FDriftNoise CylRand;
	CylRand.S = 0xC0FFEE11u;
	for (float& G : CylGain)
	{
		G = 1.f + 0.06f * CylRand.Next();
	}
	for (float& H : Harm)
	{
		H = 0.f;
	}
	for (int32 W = 0; W < 4; ++W)
	{
		FWheelVoice& V = Wheels[W];
		V.Squeal.Init(0.035f, Rate);
		V.Scrub.Init(0.035f, Rate);
		V.Rumble.Init(0.05f, Rate);
		V.Pitch.Init(0.06f, Rate);
		V.Pitch.V = 1.f;
		V.Vibrato.Init(0.03f, Rate);
		V.Grain.Init(0.002f, Rate);
		V.Noise.S = 0xA5A5A5A5u + 0x9E3779B9u * static_cast<uint32>(W + 1);
		V.Noise2.S = 0x5A5A5A5Au + 0x7F4A7C15u * static_cast<uint32>(W + 1);
		V.Detune = kWheelDetune[W];
	}
}

void FDriftAudioRenderer::Render(const FDriftAudioParams& P, const FDriftAudioConfig& C, float* Out, int32 Frames, int32 Channels)
{
	const int32 Cyl = FMath::Clamp(C.Cylinders, 1, 12);
	int32 Done = 0;
	while (Done < Frames)
	{
		const int32 N = FMath::Min(kBlock, Frames - Done);
		BlockControl(P, C, Cyl);
		for (int32 I = 0; I < N; ++I)
		{
			const float E = EngineSample(P, C, Cyl);
			float L = E, R = E;
			TyreSample(C, L, R);
			L = SoftClip(L * C.MasterGain);
			R = SoftClip(R * C.MasterGain);
			float* F = Out + static_cast<int64>(Done + I) * Channels;
			if (Channels == 1)
			{
				F[0] = 0.5f * (L + R);
			}
			else
			{
				F[0] = L;
				F[1] = R;
				for (int32 Ch = 2; Ch < Channels; ++Ch)
				{
					F[Ch] = 0.f;
				}
			}
		}
		Done += N;
	}
}

void FDriftAudioRenderer::BlockControl(const FDriftAudioParams& P, const FDriftAudioConfig& C, int32 Cyl)
{
	++BlockCount;
	const float BlockSec = kBlock * InvRate;
	const float Rpm = RpmS.V, Thr = ThrS.V;
	const float RpmN = FMath::Clamp(Rpm / kRpmRef, 0.f, 1.2f);
	const float LoadPos = FMath::Max(LoadS.V, 0.f);
	const float Cut = LimEnvS.V;

	// Exhaust: fixed pipe resonances, low-pass opening up with rpm and throttle
	// (more gas flow and sharper blowdown = brighter).
	ExDc.Set(25.f, 0.707f, InvRate);
	ExRes1.Set(C.ExhaustResonanceHz, 1.8f, InvRate);
	ExRes2.Set(C.ExhaustResonance2Hz, 2.5f, InvRate);
	ExLp1.Set(450.f + 4200.f * RpmN * (0.35f + 0.65f * Thr), 0.6f, InvRate);
	ExLp2.CopyTuning(ExLp1);
	IntakeBp.Set(200.f + 0.2f * Rpm, 1.1f, InvRate);          // induction roar follows rpm
	RaspBp.Set(900.f + 1500.f * RpmN, 0.8f, InvRate);
	PopBp.Set(1100.f, 0.7f, InvRate);
	BovBp.Set(1300.f + 2400.f * BovEnv, 1.3f, InvRate);        // "pssh" falls in pitch as it empties

	// Harmonics of the half order (cycle frequency): k = Cyl is the firing
	// frequency (rpm/60 * Cyl/2). Firing harmonics grow with cylinder pressure
	// (load); the 0.5/1st orders are cylinder-to-cylinder and rotating imbalance.
	NumHarm = FMath::Min(2 * Cyl, kMaxHarm);
	for (int32 K = 0; K <= kMaxHarm; ++K)
	{
		Harm[K] = 0.f;
	}
	Harm[1] += 0.05f;
	if (NumHarm >= 2)
	{
		Harm[2] += 0.06f;
	}
	if (Cyl >= 4)
	{
		Harm[Cyl / 2] += 0.05f;
	}
	Harm[Cyl] += 0.35f * (0.3f + 0.7f * LoadPos);
	if (2 * Cyl <= NumHarm)
	{
		Harm[2 * Cyl] += 0.15f * (0.2f + 0.8f * LoadPos);
	}
	const float HarmCut = 1.f - 0.6f * Cut;
	for (int32 K = 1; K <= NumHarm; ++K)
	{
		Harm[K] *= HarmCut;
	}

	// Turbo: compressor speed ~ sqrt(pressure rise) (centrifugal compressor).
	const float Shaft = FMath::Clamp(FMath::Sqrt(FMath::Max(BoostS.V, 0.f) / C.TurboRefBar), 0.f, 1.2f);
	TurboHz = C.TurboMinHz + (C.TurboMaxHz - C.TurboMinHz) * Shaft;
	TurboBp.Set(TurboHz, 16.f, InvRate);
	TurboLvlT = C.TurboGain * Shaft * Shaft * (0.35f + 0.65f * Thr);

	WhineT = (C.GearWhineGain > 0.f && P.Gear != 0) ? C.GearWhineGain * (0.2f + FMath::Abs(LoadS.V)) * RpmN : 0.f;

	// Overrun crackle is strongest just after lifting, then dies out.
	const bool bOverrun = LoadS.V < -0.02f && Thr < 0.05f && Rpm > 2500.f;
	OverrunTime = bOverrun ? OverrunTime + BlockSec : 0.f;

	// Rolling road + wind, ~speed^1.5.
	const float Speed = FMath::Max(P.SpeedMs, 0.f);
	RoadT = C.RoadGain * FMath::Pow(FMath::Min(Speed / 40.f, 1.5f), 1.5f);
	RoadLp.Set(120.f + 8.f * Speed, 0.7f, InvRate);

	// Tyres: combined slip sqrt(sr^2 + tan(sa)^2) above a threshold, weighted by load.
	const float SlipThr = C.TyreSlipThreshold, SlipFull = FMath::Max(C.TyreSlipFull, SlipThr + 0.01f);
	for (int32 Wi = 0; Wi < 4; ++Wi)
	{
		const FDriftAudioWheel& W = P.Wheels[Wi];
		FWheelVoice& V = Wheels[Wi];
		const float TanA = FMath::Tan(FMath::Clamp(W.SlipAngle, -1.4f, 1.4f));
		const float Comb = FMath::Min(FMath::Sqrt(W.SlipRatio * W.SlipRatio + TanA * TanA), 3.f);
		const float LoadF = W.Load < 150.f ? 0.f : FMath::Pow(FMath::Min(W.Load / C.TyreRefLoad, 1.8f), 0.7f);
		// Sliding needs speed - car speed, or wheel spin (burnout from rest).
		const float Slide = FMath::Clamp(FMath::Max(Speed, 8.f * FMath::Min(FMath::Abs(W.SlipRatio), 1.f)) / 4.f, 0.f, 1.f);
		if (W.bOnRoad)
		{
			V.SquealT = FMath::SmoothStep(SlipThr, SlipFull, Comb) * LoadF * Slide;
			V.ScrubT = FMath::SmoothStep(0.5f * SlipThr, SlipFull, Comb) * LoadF * Slide;   // scrub starts before squeal
			V.RumbleT = 0.f;
		}
		else
		{
			V.SquealT = V.ScrubT = 0.f;
			V.RumbleT = LoadF * FMath::Clamp(Speed / 15.f, 0.f, 1.2f) * (0.5f + 0.5f * FMath::SmoothStep(0.5f * SlipThr, SlipFull, Comb));
		}
		V.PitchT = 1.f + 0.3f * FMath::Clamp((Comb - SlipThr) / (1.f - FMath::Min(SlipThr, 0.9f)), 0.f, 1.f);
		V.ToneMix = 1.f - 0.5f * FMath::SmoothStep(0.6f, 1.5f, Comb);    // huge slip: more hiss than tone
		const float Pan = FMath::Clamp(W.Pan * C.TyreStereoWidth, -1.f, 1.f);
		const float Ang = (Pan + 1.f) * (0.25f * UE_PI);
		V.GainL = UE_SQRT_2 * FMath::Cos(Ang);   // equal power, 1/1 at the centre
		V.GainR = UE_SQRT_2 * FMath::Sin(Ang);
		if ((BlockCount & 31u) == 0u)
		{
			V.VibT = V.Noise2.Next();                // ~20 ms random wobble of the squeal
		}
		const float G = V.Noise2.Next01();
		V.GrainT = G * G * G;                        // sparse gravel hits
		V.bActive = V.SquealT + V.ScrubT + V.RumbleT > 1e-4f || V.Squeal.V + V.Scrub.V + V.Rumble.V > 1e-4f;
		if (V.bActive)
		{
			V.SquealBp.Set(C.SquealHz * V.Detune * V.Pitch.V, 7.f, InvRate);
			V.ScrubBp.Set(FMath::Min(300.f + 12.f * Speed, 900.f), 0.9f, InvRate);
			V.RumbleLp.Set(FMath::Min(80.f + 5.f * Speed, 400.f), 0.7f, InvRate);
			V.CrunchBp.Set(2400.f, 1.f, InvRate);
		}
	}
}

void FDriftAudioRenderer::NewFiring(int32 Slot, const FDriftAudioConfig& C)
{
	const float Rpm = RpmS.V, Load = LoadS.V;
	// Idle is lumpy (uneven mixture/combustion); high rpm is smooth.
	const float Lumpy = 1.f - 0.65f * FMath::SmoothStep(900.f, 3500.f, Rpm);
	const float LoadPos = FMath::Max(Load, 0.f);
	// Exhaust pulse strength ~ cylinder pressure at blowdown: load. Overrun and
	// fuel cut leave only the pumping pulse (but at idle the idle control still fires every cylinder).
	float Combustion = Load >= 0.f ? 0.25f + 0.75f * LoadPos : FMath::Lerp(0.25f, 0.12f, FMath::SmoothStep(1100.f, 2000.f, Rpm));
	if (bCutting)
	{
		Combustion = 0.08f;
	}
	FireAmp = Combustion * CylGain[Slot] * (1.f + 0.1f * Lumpy * FireNoise.Next());
	FireDelay = 0.03f + 0.025f * Lumpy * FireNoise.Next();   // timing scatter, fraction of a slot
	// Blowdown takes roughly fixed crank degrees; sharper (narrower) at high pressure = brighter.
	FireInvWidth = 1.f / FMath::Lerp(0.55f, 0.3f, LoadPos);

	if (C.CrackleGain > 0.f)
	{
		float Chance = 0.f;
		if (bCutting)
		{
			Chance = 0.25f;   // unburnt fuel lighting in the hot exhaust
		}
		else if (OverrunTime > 0.f)
		{
			Chance = 0.06f * FMath::SmoothStep(2500.f, 6500.f, Rpm) * FMath::Exp(-OverrunTime / 1.5f);
		}
		if (Chance > 0.f && PopNoise.Next01() < Chance)
		{
			PopEnv = FMath::Max(PopEnv, 0.4f + 0.6f * PopNoise.Next01());
		}
	}
}

float FDriftAudioRenderer::EngineSample(const FDriftAudioParams& P, const FDriftAudioConfig& C, int32 Cyl)
{
	const float Rpm = RpmS.Tick(FMath::Max(P.Rpm, 0.f));
	const float Thr = ThrS.Tick(FMath::Clamp(P.Throttle, 0.f, 1.f));
	LoadS.Tick(FMath::Clamp(P.Load, -1.f, 1.f));
	const float Boost = BoostS.Tick(FMath::Max(P.BoostBar, 0.f));
	const float ThrFast = ThrFastS.Tick(P.Throttle);
	const float ThrSlow = ThrSlowS.Tick(P.Throttle);

	// Limiter: fuel cut for kLimiterDuty of each period, starting the moment it hits.
	if (P.bLimiter)
	{
		LimPhase = bLimiterWas ? Wrap01(LimPhase + C.LimiterHz * InvRate) : 0.f;
	}
	bLimiterWas = P.bLimiter;
	bCutting = P.bLimiter && LimPhase < kLimiterDuty;
	const float Cut = LimEnvS.Tick(bCutting ? 1.f : 0.f);

	// Blow-off: a fast throttle lift (fast follower drops well below the slow one) with boost up.
	if (bBovArmed && ThrSlow - ThrFast > 0.3f && Boost > C.BlowOffMinBar)
	{
		BovEnv = FMath::Clamp(Boost / C.TurboRefBar, 0.3f, 1.f);
		bBovArmed = false;
	}
	else if (!bBovArmed && ThrFast > 0.6f)
	{
		bBovArmed = true;
	}
	BovEnv = BovEnv > 1e-5f ? BovEnv * BovDecay : 0.f;
	const float Bov = BovAtkS.Tick(BovEnv);

	// Engine off/stalled: fade below ~450 rpm instead of clicking.
	const float On = FMath::SmoothStep(150.f, 450.f, Rpm);
	if (On <= 0.f)
	{
		return 0.f;
	}

	// Crank: one cycle = two revolutions, Cyl evenly spaced firings per cycle.
	CyclePhase = Wrap01(CyclePhase + Rpm * (1.f / 120.f) * InvRate);
	const float SlotF = CyclePhase * Cyl;
	const int32 Slot = FMath::Min(static_cast<int32>(SlotF), Cyl - 1);
	if (Slot != LastSlot)
	{
		LastSlot = Slot;
		NewFiring(Slot, C);
	}
	// Exhaust pressure pulse: smooth bump (16 x^2 (1-x)^2) - band-limited enough at 8000 rpm.
	const float X = (SlotF - Slot - FireDelay) * FireInvWidth;
	float Shape = 0.f;
	if (X > 0.f && X < 1.f)
	{
		const float Y = X * (1.f - X);
		Shape = 16.f * Y * Y;
	}
	const float Pulse = Shape * FireAmp;

	// Additive half-order harmonics via the Chebyshev recurrence (one sincos per sample).
	float S1, C1;
	FMath::SinCos(&S1, &C1, UE_TWO_PI * CyclePhase);
	float Prev = 0.f, Cur = S1, Add = Harm[1] * S1;
	for (int32 K = 2; K <= NumHarm; ++K)
	{
		const float Next = 2.f * C1 * Cur - Prev;
		Prev = Cur;
		Cur = Next;
		Add += Harm[K] * Cur;
	}

	// Crackle: a short noise burst; part of it goes through the pipe as a thump.
	const float PopN = PopNoise.Next();
	PopEnv = PopEnv > 1e-5f ? PopEnv * PopDecay : 0.f;
	const float Pop = PopEnv * PopN * C.CrackleGain;

	// Exhaust: DC block -> pipe resonances -> 24 dB low-pass.
	float Ex = ExDc.High(Pulse + Add + 0.8f * Pop);
	Ex = 0.5f * Ex + 1.1f * ExRes1.Band(Ex) + 0.6f * ExRes2.Band(Ex);
	Ex = ExLp2.Low(ExLp1.Low(Ex));

	const float N1 = Noise.Next(), N2 = Noise2.Next();
	const float RpmN = FMath::Clamp(Rpm / kRpmRef, 0.f, 1.2f);
	// Rasp: turbulent noise riding on each exhaust pulse.
	const float Rasp = 4.f * RaspBp.Band(N1 * Pulse);
	// Intake roar: rises with throttle opening, pulsing with the induction strokes.
	const float Intake = 3.f * IntakeBp.Band(N2) * (0.65f + 0.35f * Shape);

	const float ExLevel = (0.35f + 0.65f * RpmN) * (0.55f + 0.45f * Thr);
	const float RaspLevel = C.ExhaustNoiseGain * (0.25f + 0.75f * Thr) * (0.3f + 0.7f * RpmN);
	const float IntakeLevel = C.IntakeGain * Thr * FMath::Sqrt(Thr) * (0.2f + 0.8f * RpmN) * (1.f - 0.8f * Cut);
	float Out = C.EngineGain * (0.45f * Ex * ExLevel + Rasp * RaspLevel + Intake * IntakeLevel);
	Out += 4.f * PopBp.Band(Pop);

	// Turbo whistle: a tone plus narrow noise at the compressor pitch.
	const float TurboLvl = TurboLvlS.Tick(TurboLvlT);
	if (TurboLvl > 1e-5f)
	{
		TurboPhase = Wrap01(TurboPhase + TurboHz * InvRate);
		Out += TurboLvl * (0.5f * FMath::Sin(UE_TWO_PI * TurboPhase) + 5.f * TurboBp.Band(N1));
	}
	if (Bov > 1e-5f)
	{
		Out += 3.f * C.BlowOffGain * Bov * BovBp.Band(N2);
	}
	// Straight-cut gear whine (input-shaft mesh, ~23 teeth).
	const float Whine = WhineS.Tick(WhineT);
	if (Whine > 1e-5f)
	{
		WhinePhase = Wrap01(WhinePhase + Rpm * (23.f / 60.f) * InvRate);
		Out += 0.3f * Whine * FMath::Sin(UE_TWO_PI * WhinePhase);
	}
	return On * Out;
}

void FDriftAudioRenderer::TyreSample(const FDriftAudioConfig& C, float& L, float& R)
{
	const float Road = RoadS.Tick(RoadT);
	if (Road > 1e-5f)
	{
		const float RoadOut = 3.f * Road * RoadLp.Low(Noise2.Next());
		L += RoadOut;
		R += RoadOut;
	}
	const float TyreScale = 0.5f * C.TyreGain;
	for (FWheelVoice& V : Wheels)
	{
		if (!V.bActive)
		{
			continue;
		}
		const float Sq = V.Squeal.Tick(V.SquealT);
		const float Sc = V.Scrub.Tick(V.ScrubT);
		const float Rb = V.Rumble.Tick(V.RumbleT);
		const float Pt = V.Pitch.Tick(V.PitchT);
		const float Vib = V.Vibrato.Tick(V.VibT);
		const float Gr = V.Grain.Tick(V.GrainT);
		const float N = V.Noise.Next();
		float Out = 4.f * C.ScrubGain * Sc * V.ScrubBp.Band(N);
		if (Sq > 1e-5f)
		{
			// Squeal: stick-slip tone (fundamental + 2nd harmonic) with a little
			// wobble, plus narrow-band noise at the same pitch.
			V.Phase = Wrap01(V.Phase + C.SquealHz * V.Detune * Pt * (1.f + 0.02f * Vib) * InvRate);
			float S, Co;
			FMath::SinCos(&S, &Co, UE_TWO_PI * V.Phase);
			const float Tone = S + 0.35f * 2.f * S * Co;
			Out += C.SquealGain * Sq * (1.f + 0.25f * Vib) * (0.5f * V.ToneMix * Tone + 6.f * V.SquealBp.Band(N));
		}
		if (Rb > 1e-5f)
		{
			Out += C.OffRoadGain * Rb * (6.f * V.RumbleLp.Low(N) + 3.f * Gr * V.CrunchBp.Band(V.Noise2.Next()));
		}
		Out *= TyreScale;
		L += Out * V.GainL;
		R += Out * V.GainR;
	}
}

// --- Component -------------------------------------------------------------------

UDriftEngineSynth::UDriftEngineSynth(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	NumChannels = 2;
	bAutoActivate = false;
}

bool UDriftEngineSynth::Init(int32& SampleRate)
{
	// Called from Start() (game thread) before the audio thread renders anything.
	NumChannels = 2;
	Renderer.Init(SampleRate);
	UE_LOG(LogDriftAudio, Log, TEXT("%s: procedural engine/tyre audio at %d Hz"), *GetName(), SampleRate);
	return true;
}

bool UDriftEngineSynth::StartSound(bool bLocalCar)
{
	if (bStartTried)
	{
		return IsPlaying();
	}
	bStartTried = true;
	FDriftAudioConfig C;
	C.Load();
	if (!C.bEnabled)
	{
		UE_LOG(LogDriftAudio, Log, TEXT("[DriftAudio] Enabled=False: no car sound"));
		return false;
	}
	Cfg = C;   // not rendering yet
	if (bLocalCar)
	{
		// The driver hears their own car in stereo, whatever the camera does.
		bAllowSpatialization = false;
	}
	else
	{
		bAllowSpatialization = true;
		bOverrideAttenuation = true;
		AttenuationOverrides.bAttenuate = true;
		AttenuationOverrides.bSpatialize = true;
		AttenuationOverrides.AttenuationShapeExtents = FVector(500.f, 0.f, 0.f);   // full level within 5 m
		AttenuationOverrides.FalloffDistance = 15000.f;                             // silent at 155 m
	}
	Start();
	return IsPlaying();
}

void UDriftEngineSynth::SetParams(const FDriftAudioParams& P)
{
	Slots[WriteSlot] = P;
	WriteSlot = Middle.exchange(WriteSlot | kFresh, std::memory_order_acq_rel) & 3u;
}

void UDriftEngineSynth::ReloadConfig()
{
	FDriftAudioConfig C;
	C.Load();
	SynthCommand([this, C]() { Cfg = C; });
}

int32 UDriftEngineSynth::OnGenerateAudio(float* OutAudio, int32 NumSamples)
{
	if (Middle.load(std::memory_order_relaxed) & kFresh)
	{
		ReadSlot = Middle.exchange(ReadSlot, std::memory_order_acq_rel) & 3u;
	}
	const int32 Channels = FMath::Max(NumChannels, 1);
	Renderer.Render(Slots[ReadSlot], Cfg, OutAudio, NumSamples / Channels, Channels);
	return NumSamples;
}
