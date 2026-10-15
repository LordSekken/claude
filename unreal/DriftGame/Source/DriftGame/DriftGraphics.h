// Graphics presets: Low / Medium / High / Epic / VR. Each sets Unreal's scalability
// groups plus the expensive features (Lumen GI + reflections, virtual shadow maps,
// render resolution with TSR upscaling). F5 cycles in game; the choice is saved
// to GameUserSettings.ini [DriftGraphics] Preset.
#pragma once

#include "CoreMinimal.h"

namespace DriftGraphics
{
	enum EPreset : int32 { Low, Medium, High, Epic, VR, Count };

	const TCHAR* Name(int32 Preset);
	int32 Saved();                    // last chosen preset (High if none)
	void Apply(int32 Preset, bool bSave = true);
}
